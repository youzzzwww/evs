/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include "options.h"
#include "stl.h"
#include "stat_dec_fx.h"
#include "prot_fx.h"
#include "g192.h"
#include "disclaimer.h"

#include "EvsRXlib.h"

/*------------------------------------------------------------------------------------------*
 * Global variables
 *------------------------------------------------------------------------------------------*/
long frame = 0;                 /* Counter of frames */


int main(int argc, char *argv[])
{
    Decoder_State_fx  *st_fx;                             /* decoder state structure     */
    Word16            zero_pad, dec_delay,output_frame;
    FILE              *f_stream;                          /* input bitstream file        */
    FILE              *f_synth;                           /* output synthesis file       */
    UWord16           bit_stream[MAX_BITS_PER_FRAME+16];
    Word16            output[3*L_FRAME48k];               /* buffer for output synthesis */
    char              *jbmTraceFileName = NULL;           /* VOIP tracefile name         */
    Word16            quietMode = 0;
    Word16            noDelayCmp = 0;
    char              *jbmFECoffsetFileName = NULL;       /* FEC offset file name */

    BASOP_init




    /*------------------------------------------------------------------------------------------*
     * Allocation of memory for static variables
     *   - I/O initializations
     *   - Decoder variables initialization
     *   - Find frame length
     *------------------------------------------------------------------------------------------*/

    if ( (st_fx = (Decoder_State_fx *) calloc(1, sizeof(Decoder_State_fx) ) ) == NULL )
    {
    }

    /*------------------------------------------------------------------------------------------*
     * I/O initializations
     * Decoder variables initialization
     *------------------------------------------------------------------------------------------*/

    st_fx->bit_stream_fx = bit_stream;

    io_ini_dec_fx( argc, argv, &f_stream, &f_synth,
                   &quietMode,
                   &noDelayCmp,
                   st_fx,
                   &jbmTraceFileName
                   ,&jbmFECoffsetFileName
                 );

    /*output_frame = (short)(st_fx->output_Fs / 50);*/
    st_fx->output_frame_fx = extract_l(Mult_32_16(st_fx->output_Fs_fx , 0x0290));

    init_decoder_fx(st_fx);
    srand((unsigned int)time(0));

    reset_indices_dec_fx(st_fx);

    IF(st_fx->Opt_VOIP_fx)
    {
        IF( decodeVoip(st_fx, f_stream, f_synth, jbmTraceFileName, jbmFECoffsetFileName  ) != 0 )

        {
            return -1;
        }
    }
    ELSE
    {
        /*------------------------------------------------------------------------------------------*
         * Regular EVS decoder with ITU-T G.192 bitstream
         *------------------------------------------------------------------------------------------*/

        /* output frame length */
        output_frame = st_fx->output_frame_fx;

        if( noDelayCmp == 0)
        {
            /* calculate the compensation (decoded signal aligned with original signal) */
            /* the number of first output samples will be reduced by this amount */
            dec_delay = NS2SA_fx2(st_fx->output_Fs_fx, get_delay_fx(DEC, st_fx->output_Fs_fx));
        }
        else
        {
            dec_delay = 0;
        }
        zero_pad = dec_delay;

        /*------------------------------------------------------------------------------------------*
         * Loop for every packet (frame) of bitstream data
         * - Read the bitstream packet
         * - Run the decoder
         * - Write the synthesized signal into output file
         *------------------------------------------------------------------------------------------*/
        if (quietMode == 0)
        {
            fprintf( stdout, "\n------ Running the decoder ------\n\n" );
            fprintf( stdout, "Frames processed:       " );
        }
        else {
            fprintf( stdout, "\n-- Start the decoder (quiet mode) --\n\n" );
        }
        BASOP_end_noprint;
        BASOP_init;
#if (WMOPS)
        Init_WMOPS_counter();
        Reset_WMOPS_counter();
        setFrameRate(48000, 960);
#endif

        /*----- loop: decode-a-frame -----*/
        WHILE( read_indices_fx( st_fx, f_stream, 0 ) )
        {
#if (WMOPS)
            fwc();
            Reset_WMOPS_counter();
#endif


            SUB_WMOPS_INIT("evs_dec");

            /* run the main encoding routine */
            IF(sub(st_fx->codec_mode, MODE1) == 0)
            {
                IF ( st_fx->Opt_AMR_WB_fx )
                {
                    amr_wb_dec_fx( output,st_fx);
                }
                ELSE
                {
                    evs_dec_fx( st_fx, output, FRAMEMODE_NORMAL);
                }
            }
            ELSE
            {
                IF(st_fx->bfi_fx == 0)
                {
                    evs_dec_fx( st_fx, output, FRAMEMODE_NORMAL);
                }
                ELSE /* conceal */
                {
                    evs_dec_fx( st_fx, output, FRAMEMODE_MISSING);
                }
            }

            END_SUB_WMOPS;



            /* increase the counter of initialization frames */

            if( sub(st_fx->ini_frame_fx,MAX_FRAME_COUNTER) < 0 )
            {
                st_fx->ini_frame_fx = add(st_fx->ini_frame_fx,1);
            }

            /* write the synthesized signal into output file */
            /* do final delay compensation */
            IF ( dec_delay == 0 )
            {
                fwrite( output, sizeof(Word16), output_frame, f_synth );
            }
            ELSE
            {
                IF ( sub(dec_delay , output_frame) <= 0 )
                {
                    fwrite( output +dec_delay, sizeof(Word16), sub(output_frame , dec_delay), f_synth );
                    dec_delay = 0;
                    move16();
                }
                ELSE
                {
                    dec_delay = sub(dec_delay, output_frame);
                }
            }
            if (quietMode == 0)
            {
                fprintf( stdout, "%-8ld\b\b\b\b\b\b\b\b", frame);
            }
            frame++;
        }

        /*----- decode-a-frame-loop end -----*/

        fflush( stderr );
        printf("Decoding finished");
        fprintf( stdout, "\n\n" );
        fflush(stdout);


        fflush(stdout);
        fflush(stderr);

        /* end of WMOPS counting */
#if (WMOPS)
        fwc();
        printf("\nDecoder complexity\n");
        WMOPS_output(0);
        printf("\n");
#endif

        /* add zeros at the end to have equal length of synthesized signals */
        set16_fx( output, 0, zero_pad );
        fwrite( output, sizeof(Word16), zero_pad, f_synth );
        BASOP_init
        destroy_decoder( st_fx );
        BASOP_end_noprint
    }

    /* free memory etc. */
    free( st_fx );
    fclose( f_synth );
    fclose( f_stream );


    return 0;
}
