/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* VMR-WB compilation switches            */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_enc_fx.h"    /* Encoder static table prototypes        */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"


/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define STEP            4
#define MSIZE           256
#define NB_MAX          8

/*-------------------------------------------------------------------*
 * Local function prototypes
 *-------------------------------------------------------------------*/
static Word16 quant_1p_N1_fx(const Word16 pos,const Word16 N);
static Word16 quant_2p_2N1_fx(const Word16 pos1, const Word16 pos2, const Word16 N);
static Word16 quant_3p_3N1_fx( const Word16 pos1, const Word16 pos2, const Word16 pos3, const Word16 N );
static Word32 quant_4p_4N_fx( const Word16 pos[], const Word16 N );
static Word32 quant_5p_5N_fx( const Word16 pos[], const Word16 N );
static Word32 quant_6p_6N_2_fx( const Word16 pos[], const Word16 N );



static Word32 fcb_encode_position_fx(Word16 pos_vector[],Word32 n,Word32 pos_num,Word32 flag);
static Word32 fcb_encode_class_fx(Word32 sector_6p_num[],Word32 pulse_num,Word32 pulse_pos_num);
static Word32 fcb_encode_PI_fx(const Word16 v[], Word32 pulse_num);
static Word32 pre_process_fx( const Word16 v[], Word16 sector_6p[], Word32 sector_6p_num[], Word32 *pulse_pos_num );

/*---------------------------------------------------------------------*
  * ACELP_4t64()
  *
  * 20, 36, 44, 52, 64, 72, 88 bits algebraic codebook.
  * 4 tracks x 16 positions per track = 64 samples.
  *
  * 20 bits --> 4 pulses in a frame of 64 samples.
  * 36 bits --> 8 pulses in a frame of 64 samples.
  * 44 bits 13 + 9 + 13 + 9 --> 10 pulses in a frame of 64 samples.
  * 52 bits 13 + 13 + 13 + 13 --> 12 pulses in a frame of 64 samples.
  * 64 bits 2 + 2 + 2 + 2 + 14 + 14 + 14 + 14 -->
  *                               16 pulses in a frame of 64 samples.
  * 72 bits 10 + 2 + 10 + 2 + 10 + 14 + 10 + 14 -->
  *                               18 pulses in a frame of 64 samples.
  * 88 bits 11 + 11 + 11 + 11 + 11 + 11 + 11 + 11 -->
  *                               24 pulses in a frame of 64 samples.
  * All pulses can have two (2) possible amplitudes: +1 or -1.
  * Each pulse can have sixteen (16) possible positions.
  *---------------------------------------------------------------------*/

Word16 acelp_4t64_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure */
    Word16 dn[],         /* i  : corr. between target and h[].                  */
    const Word16 cn[],         /* i  : residual after long term prediction            Q_new*/
    const Word16 H[],          /* i  : impulse response of weighted synthesis filter  Q12*/
    Word16 R[],          /* i  : autocorrelation values                         */
    const Word16 acelpautoc,   /* i  : autocorrealtion flag                           */
    Word16 code[],       /* o  : algebraic (fixed) codebook excitation          Q9*/
    Word16 y[],          /* o  : filtered fixed codebook excitation             Q9*/
    Word16 nbbits,       /* i  : number of bits per codebook                    */
    const Word16 cmpl_flag,    /* i  : complexity reduction flag                      */
    const Word16 Opt_AMR_WB    /* i  : flag indicating AMR-WB IO mode                 */
)
{

    Word16 i, k, index, track;
    Word32 L_index;
    Word16 ind[NPMAXPT*NB_TRACK_FCB_4T+32];        /* VE3: why +32 ???*/
    Word16 codvec[NB_PULSE_MAX+4]= {0};
    Word16 saved_bits = 0;

    PulseConfig config;
    Word16 indexing_indices[6], wordcnt, bitcnt;

    set16_fx(codvec, 0, NB_PULSE_MAX+4);
    SWITCH (nbbits)
    {

    case 20:          /* EVS/AMR-WB pulse indexing: 20 bits, 4 pulses, 4 tracks  */
        config.nbiter = 4;
        move16();   /* 4x12x16=768 loop                                        */
        config.alp = 16384;
        move16();         /* 2 in Q13*/
        config.nb_pulse = 4;
        move16();
        config.fixedpulses = 0;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 8;
        move16();
        BREAK;

    case 28:          /* EVS pulse indexing: 28 bits, 6 pulses, 4 tracks     */
        config.nbiter = 4;
        move16();    /* 4x20x16=1280 loops                                  */
        config.alp = 8192;
        move16(); /* coeff FOR sign setting                              */
        config.nb_pulse = 6;
        move16();
        config.fixedpulses = 0;
        move16();
        config.nbpos[0] = 6;
        move16();
        config.nbpos[1] = 6;
        move16();
        config.nbpos[2] = 8;
        move16();
        BREAK;

    case 36:          /* EVS/AMR-WB pulse indexing: 36 bits, 8 pulses, 4 tracks  */
        config.nbiter = 4;
        move16();    /* 4x20x16=1280 loops                                      */
        config.alp = 8192;
        move16();    /* coeff FOR sign setting                                  */
        config.nb_pulse = 8;
        move16();
        config.fixedpulses = 2;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 8;
        move16();
        config.nbpos[2] = 8;
        move16();
        BREAK;

    case 43:          /* EVS pulse indexing:    43 bits, 10 pulses, 4 tracks */
    case 44:          /* AMR-WB pulse indexing: 44 bits, 10 pulses, 4 tracks */
        config.nbiter = 4;
        move16();    /* 4x26x16=1664 loops                                  */
        config.alp = 8192;
        move16();
        config.nb_pulse = 10;
        move16();
        config.fixedpulses = 2;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 6;
        move16();
        config.nbpos[2] = 8;
        move16();
        config.nbpos[3] = 8;
        move16();
        BREAK;

    case 50:          /* EVS pulse indexing:    50 bits, 12 pulses, 4 tracks */
    case 52:          /* AMR-WB pulse indexing: 52 bits, 12 pulses, 4 tracks */
        config.nbiter = 4;
        move16();    /* 4x26x16=1664 loops                                  */
        config.alp = 8192;
        move16();
        config.nb_pulse = 12;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 6;
        move16();
        config.nbpos[2] = 8;
        move16();
        config.nbpos[3] = 8;
        move16();
        BREAK;

    case 56:          /* EVS pulse indexing:   56 bits, 14 pulses, 4 tracks  */
        config.nbiter = 3;
        move16();    /* 3x34x16=1632 loops                                  */
        config.alp = 8192;
        move16();
        config.nb_pulse = 14;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 6;
        move16();
        config.nbpos[2] = 8;
        move16();
        config.nbpos[3] = 8;
        move16();
        config.nbpos[4] = 8;
        move16();
        BREAK;

    case 62:          /* EVS pulse indexing:    62 bits, 16 pulses, 4 tracks */
    case 64:          /* AMR-WB pulse indexing: 64 bits, 16 pulses, 4 tracks */
        config.nbiter = 3;
        move16();    /* 3x36x16=1728 loops                                  */
        config.alp = 6554;
        move16();
        config.nb_pulse = 16;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 4;
        move16();
        config.nbpos[2] = 6;
        move16();
        config.nbpos[3] = 6;
        move16();
        config.nbpos[4] = 8;
        move16();
        config.nbpos[5] = 8;
        move16();
        BREAK;

    case 68:          /* EVS pulse indexing:    68 bits, 18 pulses, 4 tracks */
    case 72:          /* AMR-WB pulse indexing: 72 bits, 18 pulses, 4 tracks */
        config.nbiter = 3;
        move16();    /* 3x35x16=1680 loops                                  */
        config.alp = 6144;
        move16();
        config.nb_pulse = 18;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 2;
        move16();
        config.nbpos[1] = 3;
        move16();
        config.nbpos[2] = 4;
        move16();
        config.nbpos[3] = 5;
        move16();
        config.nbpos[4] = 6;
        move16();
        config.nbpos[5] = 7;
        move16();
        config.nbpos[6] = 8;
        move16();
        BREAK;

    case 73:          /* EVS pulse indexing:   73 bits, 20 pulses, 4 tracks  */
        config.nbiter = 2;
        move16();    /* 2x50x16=1600 loops                                  */
        config.alp = 6144;
        move16();
        config.nb_pulse = 20;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 4;
        move16();
        config.nbpos[2] = 6;
        move16();
        config.nbpos[3] = 6;
        move16();
        config.nbpos[4] = 6;
        move16();
        config.nbpos[5] = 8;
        move16();
        config.nbpos[6] = 8;
        move16();
        config.nbpos[7] = 8;
        move16();
        BREAK;

    case 78:          /* EVS pulse indexing:   78 bits, 22 pulses, 4 tracks  */
        config.nbiter = 2;
        move16();    /* 2x52x16=1664 loops                                  */
        config.alp = 6144;
        move16();
        config.nb_pulse = 22;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 2;
        move16();
        config.nbpos[1] = 3;
        move16();
        config.nbpos[2] = 4;
        move16();
        config.nbpos[3] = 5;
        move16();
        config.nbpos[4] = 6;
        move16();
        config.nbpos[5] = 8;
        move16();
        config.nbpos[6] = 8;
        move16();
        config.nbpos[7] = 8;
        move16();
        config.nbpos[8] = 8;
        move16();
        BREAK;

    case 88:          /* AMR-WB pulse indexing: 88 bits, 24 pulses, 4 tracks */
    case 83:          /* EVS pulse indexing:    83 bits, 24 pulses, 4 tracks */
        config.nbiter = 2;
        move16();    /* 2x53x16=1696 loop                                   */
        config.alp = 4096;
        move16();
        config.nb_pulse = 24;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 2;
        move16();
        config.nbpos[1] = 2;
        move16();
        config.nbpos[2] = 3;
        move16();
        config.nbpos[3] = 4;
        move16();
        config.nbpos[4] = 5;
        move16();
        config.nbpos[5] = 6;
        move16();
        config.nbpos[6] = 7;
        move16();
        config.nbpos[7] = 8;
        move16();
        config.nbpos[8] = 8;
        move16();
        config.nbpos[9] = 8;
        move16();
        BREAK;

    case 87:          /* EVS pulse indexing:   87 bits, 26 pulses, 4 tracks  */
        config.nbiter = 1;
        move16();
        config.alp = 4096;
        move16();
        config.nb_pulse = 26;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 6;
        move16();
        config.nbpos[2] = 6;
        move16();
        config.nbpos[3] = 8;
        move16();
        config.nbpos[4] = 8;
        move16();
        config.nbpos[5] = 8;
        move16();
        config.nbpos[6] = 8;
        move16();
        config.nbpos[7] = 8;
        move16();
        config.nbpos[8] = 8;
        move16();
        config.nbpos[9] = 8;
        move16();
        config.nbpos[10] = 8;
        move16();
        BREAK;

    case 92:          /* EVS pulse indexing:   92 bits, 28 pulses, 4 tracks  */
        config.nbiter = 1;
        move16();
        config.alp = 4096;
        move16();
        config.nb_pulse = 28;
        move16();
        config.fixedpulses = 4;
        move16();
        config.nbpos[0] = 4;
        move16();
        config.nbpos[1] = 6;
        move16();
        config.nbpos[2] = 6;
        move16();
        config.nbpos[3] = 8;
        move16();
        config.nbpos[4] = 8;
        move16();
        config.nbpos[5] = 8;
        move16();
        config.nbpos[6] = 8;
        move16();
        config.nbpos[7] = 8;
        move16();
        config.nbpos[8] = 8;
        move16();
        config.nbpos[9] = 8;
        move16();
        config.nbpos[10] = 8;
        move16();
        config.nbpos[11] = 8;
        move16();
        BREAK;
    }

    /* reduce the number of iterations as a compromise between the performance and complexity */
    if( cmpl_flag > 0 )
    {
        config.nbiter = cmpl_flag;
        move16();
    }

    config.codetrackpos = TRACKPOS_FIXED_FIRST;
    move16();
    config.bits = nbbits;
    move16();

    IF (acelpautoc)
    {
        E_ACELP_4tsearchx( dn, cn, R, code, &config, ind );

        /* Generate weighted code */
        E_ACELP_weighted_code(code, H, 12, y);
    }
    ELSE
    {
        E_ACELP_4tsearch(dn, cn, H, code, &config, ind, y);

        FOR(i=0; i<L_SUBFR; i++)
        {
            y[i] = shr(y[i],3);
            move16();/*Q9              */
        }
    }

    /*-----------------------------------------------------------------*
     * Indexing
     *-----------------------------------------------------------------*/

    IF (!Opt_AMR_WB)
    {
        saved_bits = E_ACELP_indexing( code, &config, NB_TRACK_FCB_4T, indexing_indices);

        saved_bits = 0;
        move16();

        wordcnt = shr(nbbits, 4);
        bitcnt = s_and(nbbits, 15);
        FOR ( i = 0; i < wordcnt; i++ )
        {
            push_indice_fx( st_fx, IND_ALG_CDBK_4T64, indexing_indices[i], 16 );
        }
        IF ( bitcnt )
        {
            push_indice_fx( st_fx, IND_ALG_CDBK_4T64, indexing_indices[i], bitcnt );
        }
    }
    ELSE
    {
        /* AMR-WB pulse indexing */

        IF(sub(nbbits,20) == 0)
        {
            FOR (track = 0; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track, NPMAXPT);
                index = quant_1p_N1_fx(ind[k], 4);
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64, index, 5 );
            }
        }
        ELSE IF (sub(nbbits,36) == 0)
        {
            FOR (track = 0; track < NB_TRACK_FCB_4T; track++)
            {

                k = i_mult2(track, NPMAXPT); /* k = track * NPMAXPT;*/
                index = quant_2p_2N1_fx(ind[k], ind[k+1], 4);
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64, index, 9 );
            }
        }
        ELSE IF (sub(nbbits,44) == 0)    /* AMR-WB pulse indexing */
        {
            FOR (track = 0; track < (NB_TRACK_FCB_4T - 2); track++)
            {
                k = i_mult2(track, NPMAXPT);
                index = quant_3p_3N1_fx(ind[k], ind[k+1], ind[k+2], 4);
                push_indice_fx(st_fx, IND_ALG_CDBK_4T64, index, 13 );
            }

            FOR (track = 2; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track, NPMAXPT);
                index = quant_2p_2N1_fx(ind[k], ind[k+1], 4);
                push_indice_fx(st_fx, IND_ALG_CDBK_4T64, index, 9 );
            }
        }
        ELSE IF (sub(nbbits,52) == 0)    /* AMR-WB pulse indexing */
        {
            FOR (track = 0; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track,NPMAXPT);
                index = quant_3p_3N1_fx(ind[k], ind[k+1], ind[k+2], 4);
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64, index, 13 );
            }
        }
        ELSE IF (sub(nbbits,64) == 0)    /* AMR-WB pulse indexing */
        {
            FOR (track = 0; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track, NPMAXPT);
                L_index = quant_4p_4N_fx(&ind[k], 4);
                index =  extract_l(L_shr(L_index, 14) & 3);
                logic16();
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64_1, index, 2 );
            }

            FOR (track = 0; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track, NPMAXPT);
                L_index = quant_4p_4N_fx(&ind[k], 4);
                index = extract_l(L_index & 0x3FFF);
                logic16();
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64_2, index, 14 );
            }
        }
        ELSE IF (sub(nbbits,72) == 0)
        {
            FOR(track=0; track< (NB_TRACK_FCB_4T - 2); track++)
            {
                k = i_mult2(track, NPMAXPT);
                L_index = quant_5p_5N_fx(&ind[k], 4);
                index = extract_l(L_shr(L_index, 10) & 0x03FF);
                logic16();
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64_1, index, 10 );
            }

            FOR(track = 2; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track, NPMAXPT);
                L_index = quant_4p_4N_fx(&ind[k], 4);
                index = extract_l(L_shr(L_index, 14) & 3);
                logic16();
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64_1, index, 2 );
            }

            FOR(track=0; track< (NB_TRACK_FCB_4T - 2); track++)
            {
                k = i_mult2(track, NPMAXPT);
                L_index = quant_5p_5N_fx(&ind[k], 4);
                index = extract_l(L_index & 0x03FF);
                logic16();
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64_2, index, 10 );
            }

            FOR(track = 2; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track, NPMAXPT);
                L_index = quant_4p_4N_fx(&ind[k], 4);
                index = extract_l(L_index & 0x3FFF);
                logic16();
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64_2, index, 14 );
            }
        }
        ELSE IF (sub(nbbits,88) == 0)
        {
            FOR (track = 0; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track, NPMAXPT);
                L_index = quant_6p_6N_2_fx(&ind[k], 4);
                index = extract_l(L_shr(L_index, 11) & 0x07FF);
                logic16();
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64_1, index, 11 );
            }

            FOR (track = 0; track < NB_TRACK_FCB_4T; track++)
            {
                k = i_mult2(track, NPMAXPT);
                L_index = quant_6p_6N_2_fx(&ind[k], 4);
                index = extract_l(L_index & 0x07FF);
                logic16();
                push_indice_fx( st_fx, IND_ALG_CDBK_4T64_2, index, 11 );
            }
        }
    }

    return saved_bits;
}



/*---------------------------------------------------------------------*
*encode class for 3p 4p 5p 6p/track                                   *
*---------------------------------------------------------------------*/

static Word32 fcb_encode_cl_fx(/* o:   class index of the pulse on a track       */
    Word32 buffer[],      /* i:   pulses on a track                         */
    Word32 pulse_num,     /* i:   pulses number on a track                  */
    Word32 pos_num        /* i:   number of the position which have pulse   */
)
{
    Word32  i,k;
    Word32  temp1,temp2;

    temp1 = L_sub(L_add(pos_num,pulse_num),1);
    temp2 = L_add(pulse_num, 0);

    k = L_sub(PI_select_table_fx[temp1][pulse_num],1);
    temp1 = L_sub(temp1,1);

    FOR (i = 0; i < pulse_num; i++)
    {
        k = L_sub(k,PI_select_table_fx[L_sub(temp1,buffer[i])][temp2--]);
        temp1 = L_sub(temp1,1);
    }

    return k;
}


/*---------------------------------------------------------------------*
  *encode fcb pulse index                                               *
  *---------------------------------------------------------------------*/
static Word32 fcb_encode_PI_fx( /* o:   return index of the  pulse on a track */
    const Word16 v[],      /* i:   pulse on a track                      */
    Word32 pulse_num       /* i:   number of the pulse on a track        */
)
{
    Word16  sector_p[7];
    Word32  pulse_pos_num;
    Word32  sector_p_num[7];
    Word32  code_index;
    Word32  sign;

    /*order the pulse position*/
    sign = pre_process_fx( v, sector_p, sector_p_num, &pulse_pos_num);

    /*encode the position*/
    code_index  = fcb_encode_position_fx(sector_p,16,pulse_pos_num,1);
    /*encode the class and compute class offset*/
    code_index = L_add(code_index, fcb_encode_class_fx(sector_p_num,pulse_num,pulse_pos_num));

    code_index = L_add(PI_offset_fx[pulse_num][L_sub(L_add(pulse_num, 1L), pulse_pos_num)],  L_add(L_shl(code_index, extract_l(pulse_pos_num)), sign));

    return code_index;
}


/*---------------------------------------------------------------------*
  *encode the class and compute class offset                            *
  *---------------------------------------------------------------------*/
static Word32 fcb_encode_class_fx(/* o:   class offset        */
    Word32 sector_6p_num[],   /* i:   position which have pulse on a track             */
    Word32 pulse_num,         /* i:   pulse number on a track                          */
    Word32 pulse_pos_num      /* i:   number of position which have pulse on a track   */
)
{
    Word32 i,j,k;
    Word32 mn9_offet;
    Word32 vector_class[6];
    Word32 *vector_class_ptr;

    mn9_offet = L_deposit_l(0);

    IF ( L_sub(pulse_pos_num,pulse_num) <  0 )
    {
        vector_class_ptr = vector_class;
        FOR (i = 0; i < pulse_pos_num; i++)
        {
            FOR (j = 0; j < (sector_6p_num[i]-1); j++)
            {

                *vector_class_ptr++ = i ;
                move32();
            }
        }
        k = fcb_encode_cl_fx(vector_class,L_sub(pulse_num,pulse_pos_num),pulse_pos_num);
        FOR (i = 0; i < k; i++)
        {
            mn9_offet = L_add(mn9_offet, PI_factor_fx[pulse_pos_num]);
        }
    }
    return mn9_offet;
}
/*---------------------------------------------------------------------*
 *encode the position                                                  *
 *---------------------------------------------------------------------*/
static Word32 fcb_encode_position_fx( /* o:   return index of the positions which have pulse*/
    Word16 pos_vector[],         /* i:   position of the pulse on a track              */
    Word32 n,
    Word32 pos_num,                /* i:   the number of position which have pulse   */
    Word32 flag
)
{
    Word32 i;
    Word32 mmm1;
    Word32 temp2;
    mmm1 = L_sub(PI_select_table_fx[n][pos_num], 1);
    temp2 = pos_num;
    move16();

    IF (flag)        /* no decrease */
    {
        move16(); /*ptr init*/
        FOR (i=0; i<pos_num; i++)
        {
            /*mmm1 -= PI_select_table_fx[sub(n,pos_vector[i]-1)][temp2--];*/
            mmm1 = L_sub(mmm1, PI_select_table_fx[L_sub(n,add(pos_vector[i],1))][temp2--]);
        }
    }
    ELSE
    {
        move16(); /*ptr init*/
        FOR (i=0; i<pos_num; i++)
        {
            /*mmm1 -= PI_select_table_fx[n-pos_vector[i]-1][temp2--];*/
            mmm1 = L_sub(mmm1, PI_select_table_fx[L_sub(n,add(pos_vector[i],1))][temp2--]);
            n = L_sub(n,1);
        }
    }

    return mmm1;
}

/*-------------------------------------------------------------------*
 * search_ixiy
 *
 * Find the best positions of 2 pulses in a subframe
 *-------------------------------------------------------------------*/

/*------------------------------------------------------------*
 * quant_1p_N1
 *
 * Quantization of 1 pulse with N+1 bits:
 *-------------------------------------------------------------*/
static Word16 quant_1p_N1_fx(  /* o  : return N+1 bits             */
    const Word16 pos,       /* i  : position of the pulse       */
    const Word16 N)         /* i  : number of bits FOR position */
{
    Word16 mask;
    Word16 index;


    mask = sub(shl(1, N), 1);              /* mask = ((1<<N)-1) */

    index = s_and(pos, mask);
    IF (s_and(pos, NB_POS_FCB_4T) != 0)
    {
        index = add(index, shl(1, N));   /* index += 1 << N */
    }
    return index;
}


/*------------------------------------------------------------*
 * quant_2p_2N1_fx
 *
 * Quantization of 2 pulses with 2*N+1 bits:
 *-------------------------------------------------------------*/
static Word16 quant_2p_2N1_fx(  /* o:  return (2*N)+1 bits         */
    const Word16 pos1,       /* i:  position of the pulse 1     */
    const Word16 pos2,       /* i:  position of the pulse 2     */
    const Word16 N)          /* i:  number of bits FOR position */
{
    Word16 mask, tmp;
    Word16 index;

    mask = sub(shl(1, N), 1);              /* mask = ((1<<N)-1) */

    /*----------------------------------------------------------------*
     * sign of 1st pulse == sign of 2th pulse
     *----------------------------------------------------------------*/

    logic16();
    logic16();
    IF (((pos2 ^ pos1) & NB_POS_FCB_4T) == 0)
    {
        IF (sub(pos1, pos2) <= 0)          /* ((pos1 - pos2) <= 0) */
        {
            /* index = ((pos1 & mask) << N) + (pos2 & mask) */
            index = add(shl(s_and(pos1, mask), N), s_and(pos2, mask));
        }
        ELSE
        {
            /* index = ((pos2 & mask) << N) + (pos1 & mask) */
            index = add(shl(s_and(pos2, mask), N), s_and(pos1, mask));
        }
        logic16();
        IF ((pos1 & NB_POS_FCB_4T) != 0)
        {
            tmp = shl(N, 1);
            index = add(index, shl(1, tmp)); /* index += 1 << (2*N) */
        }
    }
    /*----------------------------------------------------------------*
     * sign of 1st pulse != sign of 2th pulse
     *----------------------------------------------------------------*/
    ELSE
    {
        IF (sub(s_and(pos1, mask), s_and(pos2, mask)) <= 0)
        {
            /* index = ((pos2 & mask) << N) + (pos1 & mask); */
            index = add(shl(s_and(pos2, mask), N), s_and(pos1, mask));
            IF (s_and(pos2, NB_POS_FCB_4T) != 0)
            {
                tmp = shl(N, 1);           /* index += 1 << (2*N); */
                index = add(index, shl(1, tmp));
            }
        }
        ELSE
        {
            /* index = ((pos1 & mask) << N) + (pos2 & mask); */
            index = add(shl(s_and(pos1, mask), N), s_and(pos2, mask));
            IF (s_and(pos1, NB_POS_FCB_4T) != 0)
            {
                tmp = shl(N, 1);
                index = add(index, shl(1, tmp));    /* index += 1 << (2*N) */
            }
        }
    }

    return index;
}
/*---------------------------------------------------------------------*
 * Quantization of 3 pulses with 3*N+1 bits:                           *
 *---------------------------------------------------------------------*/
static Word16 quant_3p_3N1_fx(                       /* (o) return (3*N)+1 bits         */
    const Word16 pos1,                          /* (i) position of the pulse 1     */
    const Word16 pos2,                          /* (i) position of the pulse 2     */
    const Word16 pos3,                          /* (i) position of the pulse 3     */
    const Word16 N)                             /* (i) number of bits for position */
{
    Word16 nb_pos;
    Word16 index;

    nb_pos = shl(1, sub(N, 1));            /* nb_pos = (1<<(N-1)); */
    /*-------------------------------------------------------*
     * Quantization of 3 pulses with 3*N+1 bits:             *
     *-------------------------------------------------------*/
    logic16();
    logic16();
    logic16();
    logic16();
    IF (((pos1 ^ pos2) & nb_pos) == 0)
    {
        index = quant_2p_2N1_fx(pos1, pos2, sub(N, 1));    /* index = quant_2p_2N1_fx(pos1, pos2, (N-1)); */
        /* index += (pos1 & nb_pos) << N; */
        index = add(index, shl((Word16) (pos1 & nb_pos), N));
        logic16();
        /* index += quant_1p_N1_fx(pos3, N) << (2*N); */
        index = add(index, shl(quant_1p_N1_fx(pos3, N), shl(N, 1)));

    }
    ELSE IF (((pos1 ^ pos3) & nb_pos) == 0)
    {
        index = quant_2p_2N1_fx(pos1, pos3, sub(N, 1));    /* index = quant_2p_2N1_fx(pos1, pos3, (N-1)); */
        index = add(index, shl((Word16) (pos1 & nb_pos), N));
        logic16();
        /* index += (pos1 & nb_pos) << N; */
        index = add(index, shl(quant_1p_N1_fx(pos2, N), shl(N, 1)));
        /* index += quant_1p_N1_fx(pos2, N) <<
                                                                         * (2*N); */
    }
    ELSE
    {
        index = quant_2p_2N1_fx(pos2, pos3, sub(N, 1));    /* index = quant_2p_2N1_fx(pos2, pos3, (N-1)); */
        /* index += (pos2 & nb_pos) << N;			 */
        index = add(index, shl((Word16) (pos2 & nb_pos), N));
        logic16();
        /* index += quant_1p_N1_fx(pos1, N) << (2*N);	 */
        index = add(index, shl(quant_1p_N1_fx(pos1, N), shl(N, 1)));
    }
    return (index);
}
/*---------------------------------------------------------------------*
 * Quantization of 4 pulses with 4*N+1 bits:                           *
 *---------------------------------------------------------------------*/
static Word32 quant_4p_4N1_fx(                       /* (o) return (4*N)+1 bits         */
    const Word16 pos1,                          /* (i) position of the pulse 1     */
    const Word16 pos2,                          /* (i) position of the pulse 2     */
    const Word16 pos3,                          /* (i) position of the pulse 3     */
    const Word16 pos4,                          /* (i) position of the pulse 4     */
    const Word16 N)                             /* (i) number of bits for position */
{
    Word16 nb_pos;
    Word32 index;

    nb_pos = shl(1, sub(N, 1));            /* nb_pos = (1<<(N-1));  */
    /*-------------------------------------------------------*
     * Quantization of 4 pulses with 4*N+1 bits:             *
     *-------------------------------------------------------*/
    logic16();
    logic16();
    logic16();
    logic16();
    IF (((pos1 ^ pos2) & nb_pos) == 0)
    {
        index = quant_2p_2N1_fx(pos1, pos2, sub(N, 1));    /* index = quant_2p_2N1_fx(pos1, pos2, (N-1)); */
        /* index += (pos1 & nb_pos) << N;	 */
        index = L_add(index, L_shl(L_deposit_l((Word16) (pos1 & nb_pos)), N));
        logic16();
        /* index += quant_2p_2N1_fx(pos3, pos4, N) << (2*N); */
        index = L_add(index, L_shl(quant_2p_2N1_fx(pos3, pos4, N), shl(N, 1)));
    }
    ELSE IF (((pos1 ^ pos3) & nb_pos) == 0)
    {
        index = quant_2p_2N1_fx(pos1, pos3, sub(N, 1));
        /* index += (pos1 & nb_pos) << N; */
        index = L_add(index, L_shl(L_deposit_l((Word16) (pos1 & nb_pos)), N));
        logic16();
        /* index += quant_2p_2N1_fx(pos2, pos4, N) << (2*N); */
        index = L_add(index, L_shl(quant_2p_2N1_fx(pos2, pos4, N), shl(N, 1)));
    }
    ELSE
    {
        index = quant_2p_2N1_fx(pos2, pos3, sub(N, 1));
        /* index += (pos2 & nb_pos) << N; */
        index = L_add(index, L_shl(L_deposit_l((Word16) (pos2 & nb_pos)), N));
        logic16();
        /* index += quant_2p_2N1_fx(pos1, pos4, N) << (2*N); */
        index = L_add(index, L_shl(quant_2p_2N1_fx(pos1, pos4, N), shl(N, 1)));
    }
    return (index);
}
/*---------------------------------------------------------------------*
 * Quantization of 4 pulses with 4*N bits:                             *
 *---------------------------------------------------------------------*/

static Word32 quant_4p_4N_fx(                        /* (o) return 4*N bits             */
    const Word16 pos[],                         /* (i) position of the pulse 1..4  */
    const Word16 N)                             /* (i) number of bits for position */
{
    Word16 i, j, k, nb_pos, n_1, tmp;
    Word16 posA[4], posB[4];
    Word32 index;

    n_1 = (Word16) (N - 1);
    move16();
    nb_pos = shl(1, n_1);                  /* nb_pos = (1<<n_1); */

    i = 0;
    move16();
    j = 0;
    move16();
    FOR (k = 0; k < 4; k++)
    {
        logic16();
        IF ((pos[k] & nb_pos) == 0)
        {
            posA[i++] = pos[k];
            move16();
        }
        ELSE
        {
            posB[j++] = pos[k];
            move16();
        }
    }

    SWITCH (i)
    {
    case 0:
        tmp = sub(shl(N, 2), 3);           /* index = 1 << ((4*N)-3); */
        index = L_shl(1L, tmp);
        /* index += quant_4p_4N1_fx(posB[0], posB[1], posB[2], posB[3], n_1); */
        index = L_add(index, quant_4p_4N1_fx(posB[0], posB[1], posB[2], posB[3], n_1));
        BREAK;
    case 1:
        /* index = quant_1p_N1_fx(posA[0], n_1) << ((3*n_1)+1); */
        tmp = add(extract_l(L_shr(L_mult(3, n_1), 1)), 1);
        index = L_shl(quant_1p_N1_fx(posA[0], n_1), tmp);
        /* index += quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1); */
        index = L_add(index, quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1));
        BREAK;
    case 2:
        tmp = add(shl(n_1, 1), 1);         /* index = quant_2p_2N1_fx(posA[0], posA[1], n_1) << ((2*n_1)+1); */
        index = L_shl(quant_2p_2N1_fx(posA[0], posA[1], n_1), tmp);
        /* index += quant_2p_2N1_fx(posB[0], posB[1], n_1); */
        index = L_add(index, quant_2p_2N1_fx(posB[0], posB[1], n_1));
        BREAK;
    case 3:
        /* index = quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1) << N; */
        index = L_shl(quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1), N);
        index = L_add(index, quant_1p_N1_fx(posB[0], n_1));        /* index += quant_1p_N1_fx(posB[0], n_1); */
        BREAK;
    case 4:
        index = quant_4p_4N1_fx(posA[0], posA[1], posA[2], posA[3], n_1);
        BREAK;
    default:
        index = 0;
        fprintf(stderr, "Error in function quant_4p_4N_fx\n");
    }
    tmp = sub(shl(N, 2), 2);               /* index += (i & 3) << ((4*N)-2); */
    index = L_add(index, L_shl((L_deposit_l(i) & (3L)), tmp));
    logic16();

    return (index);
}


static Word32 quant_5p_5N_fx(                        /* (o) return 5*N bits             */
    const Word16 pos[],                         /* (i) position of the pulse 1..5  */
    const Word16 N)                             /* (i) number of bits for position */
{
    Word16 i, j, k, nb_pos, n_1, tmp;
    Word16 posA[5], posB[5];
    Word32 index, tmp2;

    n_1 = (Word16) (N - 1);
    move16();
    nb_pos = shl(1, n_1);                  /* nb_pos = (1<<n_1); */

    i = 0;
    move16();
    j = 0;
    move16();
    FOR (k = 0; k < 5; k++)
    {
        logic16();
        IF ((pos[k] & nb_pos) == 0)
        {
            posA[i++] = pos[k];
            move16();
        }
        ELSE
        {
            posB[j++] = pos[k];
            move16();
        }
    }

    SWITCH (i)
    {
    case 0:
        tmp = sub(extract_l(L_shr(L_mult(5, N), 1)), 1);        /* ((5*N)-1)) */
        index = L_shl(1L, tmp);   /* index = 1 << ((5*N)-1); */
        tmp = add(shl(N, 1), 1);  /* index += quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1) << ((2*N)+1);*/
        tmp2 = L_shl(quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1), tmp);
        index = L_add(index, tmp2);
        index = L_add(index, quant_2p_2N1_fx(posB[3], posB[4], N));        /* index += quant_2p_2N1_fx(posB[3], posB[4], N); */
        BREAK;
    case 1:
        tmp = sub(extract_l(L_shr(L_mult(5, N), 1)), 1);        /* index = 1 << ((5*N)-1); */
        index = L_shl(1L, tmp);
        tmp = add(shl(N, 1), 1);   /* index += quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1) <<((2*N)+1);  */
        tmp2 = L_shl(quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1), tmp);
        index = L_add(index, tmp2);
        index = L_add(index, quant_2p_2N1_fx(posB[3], posA[0], N));        /* index += quant_2p_2N1_fx(posB[3], posA[0], N); */
        BREAK;
    case 2:
        tmp = sub(extract_l(L_shr(L_mult(5, N), 1)), 1);        /* ((5*N)-1)) */
        index = L_shl(1L, tmp);            /* index = 1 << ((5*N)-1); */
        tmp = add(shl(N, 1), 1);           /* index += quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1) << ((2*N)+1);  */
        tmp2 = L_shl(quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1), tmp);
        index = L_add(index, tmp2);
        index = L_add(index, quant_2p_2N1_fx(posA[0], posA[1], N));        /* index += quant_2p_2N1_fx(posA[0], posA[1], N); */
        BREAK;
    case 3:
        tmp = add(shl(N, 1), 1);           /* index = quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1) << ((2*N)+1);  */
        index = L_shl(quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1), tmp);
        index = L_add(index, quant_2p_2N1_fx(posB[0], posB[1], N));        /* index += quant_2p_2N1_fx(posB[0], posB[1], N); */
        BREAK;
    case 4:
        tmp = add(shl(N, 1), 1);           /* index = quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1) << ((2*N)+1);  */
        index = L_shl(quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1), tmp);
        index = L_add(index, quant_2p_2N1_fx(posA[3], posB[0], N));        /* index += quant_2p_2N1_fx(posA[3], posB[0], N); */
        BREAK;
    case 5:
        tmp = add(shl(N, 1), 1);           /* index = quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1) << ((2*N)+1);  */
        index = L_shl(quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1), tmp);
        index = L_add(index, quant_2p_2N1_fx(posA[3], posA[4], N));        /* index += quant_2p_2N1_fx(posA[3], posA[4], N); */
        BREAK;
    default:
        index = 0;
        fprintf(stderr, "Error in function quant_5p_5N_fx\n");
    }

    return (index);
}

static Word32 quant_6p_6N_2_fx(                      /* (o) return (6*N)-2 bits         */
    const Word16 pos[],                         /* (i) position of the pulse 1..6  */
    const Word16 N)                             /* (i) number of bits for position */
{
    Word16 i, j, k, nb_pos, n_1;
    Word16 posA[6], posB[6];
    Word32 index;

    /* !!  N and n_1 are constants -> it doesn't need to be operated by Basic Operators */

    n_1 = (Word16) (N - 1);
    move16();
    nb_pos = shl(1, n_1);                  /* nb_pos = (1<<n_1); */

    i = 0;
    move16();
    j = 0;
    move16();
    FOR (k = 0; k < 6; k++)
    {
        logic16();
        IF ((pos[k] & nb_pos) == 0)
        {
            posA[i++] = pos[k];
            move16();
        }
        ELSE
        {
            posB[j++] = pos[k];
            move16();
        }
    }

    SWITCH (i)
    {
    case 0:
        index = L_shl(1L, (Word16) (6 * N - 5));        /* index = 1 << ((6*N)-5); */
        index = L_add(index, L_shl(quant_5p_5N_fx(posB, n_1), N)); /* index += quant_5p_5N_fx(posB, n_1) << N; */
        index = L_add(index, quant_1p_N1_fx(posB[5], n_1));        /* index += quant_1p_N1_fx(posB[5], n_1); */
        BREAK;
    case 1:
        index = L_shl(1L, (Word16) (6 * N - 5));        /* index = 1 << ((6*N)-5); */
        index = L_add(index, L_shl(quant_5p_5N_fx(posB, n_1), N)); /* index += quant_5p_5N_fx(posB, n_1) << N; */
        index = L_add(index, quant_1p_N1_fx(posA[0], n_1));        /* index += quant_1p_N1_fx(posA[0], n_1); */
        BREAK;
    case 2:
        index = L_shl(1L, (Word16) (6 * N - 5));        /* index = 1 << ((6*N)-5); */
        /* index += quant_4p_4N_fx(posB, n_1) << ((2*n_1)+1); */
        index = L_add(index, L_shl(quant_4p_4N_fx(posB, n_1), (Word16) (2 * n_1 + 1)));
        index = L_add(index, quant_2p_2N1_fx(posA[0], posA[1], n_1));      /* index += quant_2p_2N1_fx(posA[0], posA[1], n_1); */
        BREAK;
    case 3:
        index = L_shl(quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1), (Word16) (3 * n_1 + 1));    /* index = quant_3p_3N1_fx(posA[0], posA[1], posA[2], n_1) << ((3*n_1)+1); */
        index = L_add(index, quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1));     /* index += quant_3p_3N1_fx(posB[0], posB[1], posB[2], n_1); */
        BREAK;
    case 4:
        i = 2;
        move16();
        index = L_shl(quant_4p_4N_fx(posA, n_1), (Word16) (2 * n_1 + 1));  /* index = quant_4p_4N_fx(posA, n_1) << ((2*n_1)+1); */
        index = L_add(index, quant_2p_2N1_fx(posB[0], posB[1], n_1));      /* index += quant_2p_2N1_fx(posB[0], posB[1], n_1); */
        BREAK;
    case 5:
        i = 1;
        move16();
        index = L_shl(quant_5p_5N_fx(posA, n_1), N);       /* index = quant_5p_5N_fx(posA, n_1) << N; */
        index = L_add(index, quant_1p_N1_fx(posB[0], n_1));        /* index += quant_1p_N1_fx(posB[0], n_1); */
        BREAK;
    case 6:
        i = 0;
        move16();
        index = L_shl(quant_5p_5N_fx(posA, n_1), N);       /* index = quant_5p_5N_fx(posA, n_1) << N; */
        index = L_add(index, quant_1p_N1_fx(posA[5], n_1));        /* index += quant_1p_N1_fx(posA[5], n_1); */
        BREAK;
    default:
        index = 0;
        fprintf(stderr, "Error in function quant_6p_6N_2_fx\n");
    }
    index = L_add(index, L_shl((L_deposit_l(i) & 3L), (Word16) (6 * N - 4)));
    logic16();/* index += (i & 3) << ((6*N)-4); */

    return (index);
}


/*---------------------------------------------------------------------*
 *order the pulse position                                             *
 *---------------------------------------------------------------------*/
static Word32 pre_process_fx( /* o:   return sign value of pulse on a track              */
    const Word16 v[],          /* i:   the pulse vector                                   */
    Word16 pos_vector[],       /* o:   position of the pulse on a track                   */
    Word32 pos_vector_num[],   /* o:   the pulse number on the position which have pulse  */
    Word32 *pulse_pos_num      /* i:   the number of position which have pulse            */
)
{
    Word16  j,k;
    Word32  sign;

    sign = L_deposit_l(0);
    j = 0;
    move16();
    FOR (k=0; k<64; k+=4)
    {
        IF (v[k])
        {
            pos_vector[j] = shr(k,2);
            pos_vector_num[j] = L_shr( abs_s(v[k]), 9 );  /* Q9: 512 -> 1, Q0 */
            IF (v[k]>0)
            {
                sign = L_shl(sign, 1);
            }
            ELSE
            {
                sign = L_add(L_shl( sign, 1), 1);
            }
            j =add(j, 1);
        }
    }
    *pulse_pos_num = L_deposit_l(j);

    return sign;
}


/*--------------------------------------------------------------------------*
* E_ACELP_code43bit
*
* Fixed bit-length arithmetic coding of pulses
* v - (input) pulse vector
* s - (output) encoded state
* n - (output) range of possible states (0...n-1)
* p - (output) number of pulses found
* len - (input) length of pulse vector
* trackstep - (input) step between tracks
*--------------------------------------------------------------------------*/

Word16 E_ACELP_code43bit(const Word16 code[], UWord32 *ps, Word16 *p, UWord16 idxs[])
{
    Word16 i,j,k,track;
    Word16 ind[32];

    Word16 tmp;
    Word32 L_tmp;
    Word32 joint_index;
    static const Word32 joint_offset = 3611648;        /*offset for 3 pulses per track*/
    Word16 saved_bits = 0;

    FOR (track = 0; track < 2; track++)
    {
        ps[track] = fcb_encode_PI_fx(code+track, 3);
        move32();
        p[track] = 3;
        move16();
    }

    FOR (track = 2; track < NB_TRACK_FCB_4T; track++)
    {
        i = j = i_mult2(track, NPMAXPT);
        move16();
        FOR (k = track; k < 64; k += 4)
        {
            IF (code[k])
            {
                tmp = shr(k, 2);
                if (code[k] < 0)
                {
                    tmp = add(tmp, 16);
                }
                ind[j] = tmp;
                move16();
                IF (sub(abs_s(code[k]), 512) > 0)
                {
                    ind[j + 1] = tmp;
                    move16();
                    BREAK;
                }
                j = add(j, 1);
            }
        }
        ps[track] = quant_2p_2N1_fx(ind[i], ind[i+1], 4);
        move32();
        p[track] = 2;
        move16();
    }
    /* joint_index = ps[0]*5472 + ps[1]; */
    L_tmp = L_shl(ps[0], 12);
    L_tmp = L_add(L_tmp, L_shl(ps[0], 10));
    L_tmp = L_add(L_tmp, L_shl(ps[0], 8));
    L_tmp = L_add(L_tmp, L_shl(ps[0], 6));
    L_tmp = L_add(L_tmp, L_shl(ps[0], 5));
    joint_index = L_add(L_tmp, ps[1]);
    L_tmp = L_sub(joint_index, joint_offset);
    if (L_tmp >= 0)
    {
        joint_index = L_add(joint_index, joint_offset);
    }
    if (L_tmp < 0)
    {
        saved_bits = add(saved_bits, 1);
    }
    idxs[0] = extract_l(L_add(L_shl(ps[2], 9), ps[3]));
    idxs[1] = extract_l(L_add(L_shl(joint_index, 2), L_shr(ps[2], 7)));
    idxs[2] = extract_l(L_shr(joint_index, 14));

    return saved_bits;
}
