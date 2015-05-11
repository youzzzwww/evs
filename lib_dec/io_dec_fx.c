/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "disclaimer.h" /*for disclaimer*/
#include "basop_util.h"

/* WMC_TOOL_SKIP_FILE */

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/

static void usage_dec(void);
static char *to_upper( char *str );


/*---------------------------------------------------------------------*
 * io_ini_dec()
 *
 * Processing of command line parameters
 *---------------------------------------------------------------------*/

void io_ini_dec_fx(
    const int argc,                /* i  : command line arguments number             */
    char *argv[],             /* i  : command line arguments                    */
    FILE **f_stream,          /* o  : input bitstream file                      */
    FILE **f_synth,           /* o  : output synthesis file                     */
	FILE **f_jitter,          /* i  : jitter file                                */
	short *multi_apa,         /* i  : define how much frams once process        */
    Word16 *quietMode,             /* o  : limited printouts                         */
    Word16 *noDelayCmp,            /* o  : turn off delay compensation               */
    Decoder_State_fx *st_fx,           /* o  : Decoder static variables structure        */
	char **inputFileName,
    char **jbmTraceFileName   /* o  : VOIP tracefilename                        */
    ,char **jbmFECoffsetFileName /* : Output file  for Optimum FEC offset        */
)
{
    short i;
    char   stmp[50];

    print_disclaimer(stderr);


    /*-----------------------------------------------------------------*
     * Initialization
     *-----------------------------------------------------------------*/

    i = 1;

    *f_synth = NULL;
    *f_stream = NULL;
	*f_jitter = NULL;
    st_fx->Opt_AMR_WB_fx = 0;
    st_fx->Opt_VOIP_fx = 0;
    set_zero_Word8((Word8 *)stmp, sizeof(stmp));

    IF ( argc <= 1 )
    {
        usage_dec();
    }

    /*-----------------------------------------------------------------*
     * Optional input arguments
     *-----------------------------------------------------------------*/

    WHILE ( i < argc-3 )
    {
        /*-----------------------------------------------------------------*
         * VOIP mode
         *-----------------------------------------------------------------*/

        IF ( strcmp( to_upper(argv[i]), "-VOIP") == 0)
        {
            st_fx->Opt_VOIP_fx = 1;
            move16();
            i += 1;
        }

        /*-----------------------------------------------------------------*
         * VOIP Tracefile
         *-----------------------------------------------------------------*/

        ELSE IF ( strcmp( to_upper(argv[i]), "-TRACEFILE" ) == 0 )
        {
            *jbmTraceFileName = argv[i+1];
            i = i + 2;
        }
        /*-----------------------------------------------------------------*
        * FEC offset file
        *-----------------------------------------------------------------*/

        ELSE IF ( strcmp( to_upper(argv[i]), "-FEC_CFG_FILE" ) == 0 )
        {
            st_fx->writeFECoffset = 1;
            *jbmFECoffsetFileName = argv[i+1];
            i = i + 2;
        }

		/*-----------------------------------------------------------------*
        * jitter setting file
        *-----------------------------------------------------------------*/

        ELSE IF ( strcmp( to_upper(argv[i]), "-JITTER_FILE" ) == 0 )
        {
            *f_jitter = fopen(argv[i+1],"r");
            i = i + 2;
			if(*f_jitter == NULL)
			{
				fprintf (stderr, "Error: jitter file cann't open!\n\n");
				usage_dec();
			}
        }
		/*-----------------------------------------------------------------*
         * frames in single process
         *-----------------------------------------------------------------*/

        ELSE IF ( strcmp( to_upper(argv[i]), "-MULTI_APA" ) == 0 )
        {
            *multi_apa = atoi(argv[i+1]);
			if(*multi_apa<1 || *multi_apa>4)
				*multi_apa = 1;
            i = i+2;
        }

        /*-----------------------------------------------------------------*
         * Quiet mode
         *-----------------------------------------------------------------*/

        ELSE IF ( strcmp( to_upper(argv[i]), "-Q" ) == 0 )
        {
            *quietMode = 1;
            move16();
            i++;
        }

        /*-----------------------------------------------------------------*
         * deactivate delay compensation
         *-----------------------------------------------------------------*/

        ELSE IF ( strcmp( to_upper(argv[i]), "-NO_DELAY_CMP" ) == 0 )
        {
            *noDelayCmp = 1;
            i++;
        }


        /*-----------------------------------------------------------------*
         * Option not recognized
         *-----------------------------------------------------------------*/

        ELSE
        {
            fprintf(stderr, "Error: Unknown option %s\n\n", argv[i]);
            usage_dec();
        }

    } /* end of while  */


    /*-----------------------------------------------------------------*
     * Mandatory input arguments
     *-----------------------------------------------------------------*/

    /*-----------------------------------------------------------------*
     * Output sampling frequency
     *-----------------------------------------------------------------*/

    if( i < argc - 2 )
    {
        st_fx->output_Fs_fx = (int)atoi( argv[i] ) * 1000;
        if( st_fx->output_Fs_fx != 8000 && st_fx->output_Fs_fx != 16000 && st_fx->output_Fs_fx != 32000 && st_fx->output_Fs_fx != 48000 )
        {
            fprintf(stderr, "Error: %d kHz is not a supported sampling rate\n\n", atoi( argv[i] ) );
            usage_dec();
        }

        i++;
    }
    else
    {
        fprintf (stderr, "Error: Sampling rate is not specified\n\n");
        usage_dec();
    }

    /*-----------------------------------------------------------------*
     * Input bitstream file
     *-----------------------------------------------------------------*/

    if( i < argc - 1 )
    {
		*inputFileName = argv[i];
        if ( (*f_stream = fopen(argv[i], "rb+")) == NULL)
        {
            fprintf(stderr,"Error: input bitstream file %s cannot be opened\n\n", argv[i]);
            usage_dec();
        }

        fprintf( stderr, "Input bitstream file:   %s\n", argv[i]);
        i++;
    }
    else
    {
        fprintf (stderr, "Error: no input bitstream file specified\n\n");
        usage_dec();
    }

    /*-----------------------------------------------------------------*
     * Output synthesis file
     *-----------------------------------------------------------------*/

    if( i < argc )
    {
        if ( (*f_synth = fopen(argv[i], "wb+")) == NULL )
        {
            fprintf( stderr, "Error: ouput synthesis file %s cannot be opened\n\n", argv[i] );
            usage_dec();
        }

        fprintf( stdout, "Output synthesis file:  %s\n", argv[i] );
        i++;
    }
    else
    {
        fprintf( stderr, "Error: no output synthesis file specified\n\n" );
        usage_dec();
    }

    fprintf( stdout, "\n" );

    if( !st_fx->Opt_VOIP_fx )
    {
        /*-----------------------------------------------------------------*
         * Read information from bitstream
         *-----------------------------------------------------------------*/

        read_indices_fx( st_fx, *f_stream, 1 );

        /*-----------------------------------------------------------------*
         * Print info on screen
         *-----------------------------------------------------------------*/
        /*-----------------------------------------------------------------*
         * Print output sampling frequency
         *-----------------------------------------------------------------*/

        fprintf( stdout, "Output sampling rate:   %d Hz\n", st_fx->output_Fs_fx );

        /*-----------------------------------------------------------------*
         * Print bitrate
         *-----------------------------------------------------------------*/

        fprintf( stdout, "Bitrate:                %.2f kbps\n", (float)st_fx->total_brate_fx/1000 );

    }

    return;
}

/*---------------------------------------------------------------------*
 * to_upper()
 *
 * Capitalize all letters of a string.
 * (normally to_upper() function would be used but it does not work in Unix)
 *---------------------------------------------------------------------*/

static char *to_upper( char *str )
{
    short i;
    char *p = str;

    i = 0;
    while (str[i] != 0)
    {
        if (str[i] >= 'a' && str[i] <= 'z') str[i] -= 0x20;
        i++;
    }

    return p;
}

static void usage_dec( void )
{
    fprintf(stdout,"Usage : EVS_dec.exe [Options] Fs bitstream_file output_file\n\n");

    fprintf(stdout,"Mandatory parameters:\n");
    fprintf(stdout,"---------------------\n");
    fprintf(stdout,"Fs                : Output sampling rate in kHz (8, 16, 32 or 48)\n");
    fprintf(stdout,"bitstream_file    : Input bitstream filename or RTP packet filename (in VOIP mode)\n");
    fprintf(stdout,"output_file       : Output speech filename \n\n");

    fprintf(stdout,"Options:\n");
    fprintf(stdout,"--------\n");
    fprintf(stdout, "-VOIP            : VOIP mode,\n");
    fprintf(stdout, "-Tracefile TF    : Generate trace file named TF,\n");
    fprintf(stdout, "-no_delay_cmp    : Turn off delay compensation\n");
    fprintf(stdout, "-fec_cfg_file    : Output of the channel aware configuration. The outptut is  \n");
	fprintf(stdout, "-jitter_file     : jitter time configuration\n");
    fprintf(stdout, "                   written into a .txt file. Each line contains the FER indicator \n");
    fprintf(stdout, "                   (HI|LO) and optimum FEC offset. \n");

    fprintf(stdout, "-q               : Quiet mode, no frame counter\n");
    fprintf(stdout, "                   default is OFF\n");
	fprintf(stdout, "-multi_apa       : how much frames per extend-shrink process\n");
    fprintf(stdout, "\n");
    exit(-1);
}
