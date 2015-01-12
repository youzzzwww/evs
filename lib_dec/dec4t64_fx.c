/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "assert.h"    /* Static table prototypes                */
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local functions
 *-------------------------------------------------------------------*/
static void add_pulses_fx(const Word16 pos[],const Word16 nb_pulse, const Word16 track,Word16 code[]);
static void dec_1p_N1_fx(const  Word32 index, const Word16 N, const Word16 offset, Word16 pos[] );
static void dec_2p_2N1_fx(const Word32  index, const Word16 N, const Word16 offset, Word16 pos[] );
static void dec_3p_3N1_fx(const Word32 index, const Word16 N, const Word16 offset, Word16 pos[] );
static void dec_4p_4N1_fx(const Word32 index, const Word16 N, const Word16 offset, Word16 pos[] );
static void dec_4p_4N_fx(const Word32 index, const Word16 N, const Word16 offset, Word16 pos[]);
static void dec_5p_5N_fx(const Word32 index,const Word16 N,const Word16 offset,Word16 pos[]);
static void dec_6p_6N2_fx(const Word32 index,const Word16 N,const Word16 offset,Word16 pos[]);
static Word32 fcb_decode_PI_fx(Word32 code_index,Word16 sector_6p[],Word16 pulse_num);


/*==========================================================================*/
/* FUNCTION      : void dec_acelp_4t64_fx ()								*/
/*--------------------------------------------------------------------------*/
/* PURPOSE       :	                                                        */
/*	* 20, 36       bits algebraic codebook decoder.							*/
/*  * 4 tracks x 16 positions per track = 64 samples.						*/
/*  * 20 bits --> 4 pulses in a frame of 64 samples.						*/
/*  * 36 bits --> 8 pulses in a frame of 64 samples.						*/
/*  * All pulses can have two (2) possible amplitudes: +1 or -1.			*/
/*  * Each pulse can have sixteen (16) possible positions.					*/
/*  * See cod4t64.c for more details of the algebraic code.					*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :														*/
/* _Word16 i_subfr          i  : subframe index                             */
/* _Word16 nbbits           i  : number of bits per codebook                */
/* _Word16 FCB_5Sx4T_fla    i  : 5Sx4Track flag for PI                      */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														*/
/* _Word16 code[]			o  : algebraic (fixed) codebook excitation  Q12 */
/* _Word16 index_buf_4T[]   o  : 5Sx4Track buffer for PI                    */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														*/
/*					 _ None													*/
/*--------------------------------------------------------------------------*/
/* CALLED FROM : 															*/
/*==========================================================================*/
void dec_acelp_4t64_fx(
    Decoder_State_fx *st_fx,               /* i/o: decoder state structure */
    Word16 nbbits,         /* i  : number of bits per codebook               */
    Word16 code[],         /* o  : algebraic (fixed) codebook excitation  Q9*/
    const Word16 Opt_AMR_WB
)
{
    Word16 i, k, pos[7];
    Word32 L_index;
    Word32 ind1[NB_TRACK_FCB_4T];
    PulseConfig config;
    Word16 indexing_indices[6], wordcnt, bitcnt, index2;

    IF ( !Opt_AMR_WB )
    {
        SWITCH (nbbits)
        {
        case 20:
            config.nb_pulse = 4;
            move16();
            BREAK;

        case 28:
            config.nb_pulse = 6;
            move16();
            BREAK;

        case 36:
            config.nb_pulse = 8;
            move16();
            BREAK;

        case 43:
            config.nb_pulse = 10;
            move16();
            BREAK;

        case 50:
            config.nb_pulse = 12;
            move16();
            BREAK;

        case 56:
            config.nb_pulse = 14;
            move16();
            BREAK;

        case 62:
            config.nb_pulse = 16;
            move16();
            BREAK;

        case 68:
            config.nb_pulse = 18;
            move16();
            BREAK;

        case 73:
            config.nb_pulse = 20;
            move16();
            BREAK;

        case 78:
            config.nb_pulse = 22;
            move16();
            BREAK;

        case 83:
            config.nb_pulse = 24;
            move16();
            BREAK;

        case 87:
            config.nb_pulse = 26;
            move16();
            BREAK;

        case 92:
            config.nb_pulse = 28;
            move16();
            BREAK;
        }
        IF(nbbits == 0)
        {
            config.nb_pulse = 0;
            move16();
            FOR (i = 0; i < L_SUBFR; i++)
            {
                code[i] = 0;
                move16();
            }
        }
        ELSE
        {
            config.bits = nbbits;
            move16();
            config.codetrackpos = TRACKPOS_FIXED_FIRST;
            move16();


            wordcnt = shr(nbbits, 4);
            bitcnt = s_and(nbbits, 15);
            FOR ( i = 0; i < wordcnt; i++ )
            {
                indexing_indices[i] = get_next_indice_fx( st_fx, 16 );
                move16();
            }
            IF ( bitcnt )
            {
                indexing_indices[i] = get_next_indice_fx( st_fx, bitcnt );
                move16();
            }

            D_ACELP_indexing(code, config, 4, indexing_indices);
        }
    }
    ELSE
    {
        FOR (i=0; i<L_SUBFR; i++)
        {
            code[i] = 0;
            move16();
        }

        IF (sub(nbbits,20) == 0)
        {
            FOR (k=0; k<NB_TRACK_FCB_4T; k++)
            {
                L_index = get_next_indice_fx( st_fx, 5 );
                dec_1p_N1_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 1, k, code);
            }
        }
        ELSE IF  (sub(nbbits,36) == 0)
        {
            FOR (k=0; k<NB_TRACK_FCB_4T; k++)
            {
                L_index = get_next_indice_fx( st_fx, 9 );
                dec_2p_2N1_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 2, k, code);
            }
        }
        ELSE IF  (sub(nbbits,44) == 0)    /* AMR-WB pulse indexing */
        {
            FOR(k = 0; k < NB_TRACK_FCB_4T - 2; k++)
            {
                L_index = get_next_indice_fx( st_fx, 13 );
                dec_3p_3N1_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 3, k, code);
            }

            FOR(k = 2; k < NB_TRACK_FCB_4T; k++)
            {
                L_index = get_next_indice_fx( st_fx, 9 );
                dec_2p_2N1_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 2, k, code);
            }
        }
        ELSE IF  (sub(nbbits,52) == 0)    /* AMR-WB pulse indexing */
        {
            FOR(k = 0; k < NB_TRACK_FCB_4T; k++)
            {
                L_index = get_next_indice_fx( st_fx, 13  );
                dec_3p_3N1_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 3, k, code);
            }
        }
        ELSE IF  (sub(nbbits,64) == 0)    /* AMR-WB pulse indexing */
        {
            FOR(k = 0; k < NB_TRACK_FCB_4T; k++)
            {
                ind1[k] = get_next_indice_fx( st_fx, 2 );
            }
            FOR(k = 0; k < NB_TRACK_FCB_4T; k++)
            {
                index2 = get_next_indice_fx( st_fx, 14 );
                L_index = L_add(L_shl(ind1[k],14),index2);
                dec_4p_4N_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 4, k, code);
            }
        }
        ELSE IF  (sub(nbbits,72) == 0)
        {
            FOR(k = 0; k < NB_TRACK_FCB_4T - 2; k++)
            {
                ind1[k] = get_next_indice_fx( st_fx, 10 );
            }
            FOR(k = 2; k < NB_TRACK_FCB_4T; k++)
            {
                ind1[k] = get_next_indice_fx( st_fx, 2 );
            }
            FOR(k = 0; k < NB_TRACK_FCB_4T - 2; k++)
            {
                index2 = get_next_indice_fx( st_fx, 10 );
                L_index = L_add(L_shl(ind1[k],10) , index2);
                dec_5p_5N_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 5, k, code);
            }
            FOR(k = 2; k < NB_TRACK_FCB_4T; k++)
            {
                index2 = get_next_indice_fx( st_fx, 14 );
                L_index = L_add(L_shl(ind1[k],14) , index2);
                dec_4p_4N_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 4, k, code);
            }
        }
        ELSE IF  (sub(nbbits,88) == 0)
        {
            FOR(k = 0; k < NB_TRACK_FCB_4T; k++)
            {
                ind1[k] = get_next_indice_fx( st_fx, 11 );
                move16();
            }
            FOR(k = 0; k < NB_TRACK_FCB_4T; k++)
            {
                index2 = get_next_indice_fx( st_fx, 11 );
                L_index = L_add(L_shl(ind1[k],11) , index2);
                dec_6p_6N2_fx(L_index, 4, 0, pos);
                add_pulses_fx(pos, 6, k, code);
            }
        }
    }

    return;
}
/*-------------------------------------------------------*
  * add_pulses()
  *
  * Add decoded pulses to the codeword
  *-------------------------------------------------------*/
static void add_pulses_fx(
    const Word16 pos[],     /* i:   pulse position     */
    const Word16 nb_pulse,  /* i:   nb. of pulses      */
    const Word16 track,     /* i:   no. of the tracks  */
    Word16 code[]     /* i/o: decoded codevector */
)
{

    Word16 i,k;
    Word16 *ptr;
    ptr = code + track;
    move16();

    FOR(k=0; k<nb_pulse; k++)
    {
        /* i = ((pos[k] & (NB_POS-1))*NB_TRACK) + track */
        logic16();
        i = shl((Word16)(pos[k] & (NB_POS_FCB_4T-1)), 2);

        logic16();
        IF ((pos[k] & NB_POS_FCB_4T) == 0)
        {
            ptr[i] = add(ptr[i], 512);
            move16();
        }
        ELSE
        {
            ptr[i] = sub(ptr[i], 512);
            move16();
        }
    }
    return;
}

/*-------------------------------------------------------*
  * dec_1p_N1()
  *
  * Decode 1 pulse with N+1 bits
  *-------------------------------------------------------*/
void dec_1p_N1_fx(
    const  Word32 index,   /* i:   quantization index    */
    const Word16 N,       /* i:   nb. of bits           */
    const Word16 offset,  /* i:   pulse position offset */
    Word16 pos[]    /* o:   pulse position        */
)
{
    Word16 pos1;
    Word32 mask,i;

    mask = L_deposit_l(sub(shl(1, N), 1)); /* mask = ((1<<N)-1); */


    logic16();
    pos1 = add(extract_l(index & mask), offset);

    /* i = ((index >> N) & 1); */
    i = L_shr(index, N) & 1L;

    if (i != 0)
    {
        pos1 = add(pos1, NB_POS_FCB_4T);
    }
    pos[0] = pos1;
    move16();
}
/*-------------------------------------------------------*
 * dec_2p_2N1()
 *
 * Decode 2 pulses with 2*N+1 bits:
 *-------------------------------------------------------*/

void dec_2p_2N1_fx(
    const Word32  index,   /* i:   quantization index    */
    const Word16 N,       /* i:   nb. of bits           */
    const Word16 offset,  /* i:   pulse position offset */
    Word16 pos[]    /* o:   pulse position        */
)
{

    Word16  pos1, pos2, tmp;
    Word32  mask, i;

    mask = L_deposit_l(sub(shl(1, N), 1)); /* mask = ((1<<N)-1); */

    /* pos1 = (((index >> N) & mask) + offset); */
    logic16();
    pos1 = extract_l(L_add((L_shr(index, N) & mask), L_deposit_l(offset)));

    /* i = (index >> (2*N)) & 1; */
    tmp = shl(N, 1);
    i = L_and(L_shr(index, tmp), 1L);

    /* pos2 = ((index & mask) + offset); */
    pos2 = add(extract_l(index & mask), offset);
    logic16();


    IF (sub(pos2, pos1) < 0)
    {
        IF (i != 0)
        {
            pos1 = add(pos1, NB_POS_FCB_4T);
        }
        ELSE
        {
            pos2 = add(pos2, NB_POS_FCB_4T);
        }
    }
    ELSE
    {
        IF (i != 0)
        {
            pos1 = add(pos1, NB_POS_FCB_4T);
            pos2 = add(pos2, NB_POS_FCB_4T);
        }
    }

    pos[0] = pos1;
    move16();
    pos[1] = pos2;
    move16();

    return;

}
/*-------------------------------------------------------*
* Dec_3p_3N1
*
* Decode 3 pulses with 3*N+1 bits:
*-------------------------------------------------------*/
static void dec_3p_3N1_fx(
    const Word32 index,   /* i  : quantization index    */
    const Word16 N,       /* i  : nb. of bits           */
    const Word16 offset,  /* i  : pulse position offset */
    Word16 pos[]    /* o  : pulse position        */
)
{
    Word16 j, tmp;
    Word32 mask, idx;

    tmp = sub(shl(N, 1), 1);               /* mask = ((1<<((2*N)-1))-1); */
    mask = L_sub(L_shl(1L, tmp), 1L);

    idx = L_and(index, mask);
    j = offset;
    move16();
    tmp = sub(shl(N, 1), 1);

    logic16();
    IF ((L_shr(index, tmp) & 1L) != 0)
    {
        /* IF (((index >> ((2*N)-1)) & 1) == 1){ */
        j = add(j, shl(1, sub(N, 1)));     /* j += (1<<(N-1)); */
    }
    dec_2p_2N1_fx(idx, sub(N, 1), j, pos);

    mask = sub(shl(1, add(N, 1)), 1);      /* mask = ((1<<(N+1))-1); */
    tmp = shl(N, 1);                       /* idx = (index >> (2*N)) & mask; */
    idx = L_shr(index, tmp) & mask;
    logic16();

    dec_1p_N1_fx(idx, N, offset, pos + 2);
}

/*-------------------------------------------------------*
 * Dec_4p_4N1
 *
 * Decode 4 pulses with 4*N+1 bits:
 *-------------------------------------------------------*/
static void dec_4p_4N1_fx(
    const Word32 index,   /* i  : quantization index    */
    const Word16 N,       /* i  : nb. of bits           */
    const Word16 offset,  /* i  : pulse position offset */
    Word16 pos[]    /* o  : pulse position        */
)
{
    Word16 j, tmp;
    Word32 mask, idx;

    tmp = sub(shl(N, 1), 1);               /* mask = ((1<<((2*N)-1))-1); */
    mask = L_sub(L_shl(1L, tmp), 1L);
    idx = L_and(index, mask);
    j = offset;
    move16();
    tmp = sub(shl(N, 1), 1);

    logic16();
    IF((L_shr(index, tmp) & 1L) != 0L)
    {
        /* (((index >> ((2*N)-1)) & 1) == 1) */
        j = add(j, shl(1, sub(N, 1)));     /* j += (1<<(N-1)); */
    }
    dec_2p_2N1_fx(idx, sub(N, 1), j, pos);


    tmp = add(shl(N, 1), 1);               /* mask = ((1<<((2*N)+1))-1); */
    mask = L_sub(L_shl(1L, tmp), 1L);
    idx = L_shr(index, shl(N, 1)) & mask;
    logic16();/* idx = (index >> (2*N)) & mask; */
    dec_2p_2N1_fx(idx, N, offset, pos + 2);     /* Dec_2p_2N1(idx, N, offset, pos+2); */
}


/*-------------------------------------------------------*
* Dec_4p_4N
*
* Decode 4 pulses with 4*N bits:
*-------------------------------------------------------*/
static void dec_4p_4N_fx(
    const Word32 index,   /* i  : quantization index    */
    const Word16 N,       /* i  : nb. of bits           */
    const Word16 offset,  /* i  : pulse position offset */
    Word16 pos[]    /* o  : pulse position        */
)
{
    Word16 j, n_1, tmp;

    n_1 = sub(N, 1);
    j = add(offset, shl(1, n_1)); /* j = offset + (1 << n_1) */

    tmp = sub(shl(N, 2), 2);
    logic16();
    SWITCH (L_shr(index, tmp) & 3)
    {
        /* ((index >> ((4*N)-2)) & 3) */
    case 0:
        tmp = add(shl(n_1, 2), 1);

        logic16();
        IF((L_shr(index, tmp) & 1) == 0)
        {
            /* (((index >> ((4*n_1)+1)) & 1) == 0) */
            dec_4p_4N1_fx(index, n_1, offset, pos);
        }
        ELSE
        {
            dec_4p_4N1_fx(index, n_1, j, pos);
        }
        BREAK;
    case 1:
        tmp = add(extract_l(L_shr(L_mult(3, n_1), 1)), 1); /* Dec_1p_N1((index>>((3*n_1)+1)), n_1, offset, pos) */
        dec_1p_N1_fx(L_shr(index, tmp), n_1, offset, pos);
        dec_3p_3N1_fx(index, n_1, j, pos + 1);
        move16();
        BREAK;
    case 2:
        tmp = add(shl(n_1, 1), 1); /* Dec_2p_2N1((index>>((2*n_1)+1)), n_1, offset, pos) */
        dec_2p_2N1_fx(L_shr(index, tmp), n_1, offset, pos);
        dec_2p_2N1_fx(index, n_1, j, pos + 2);
        move16();
        BREAK;
    case 3:
        tmp = add(n_1, 1); /* Dec_3p_3N1((index>>(n_1+1)), n_1, offset, pos) */
        dec_3p_3N1_fx(L_shr(index, tmp), n_1, offset, pos);
        dec_1p_N1_fx(index, n_1, j, pos + 3);
        move16();
        BREAK;
    }
}


/*-------------------------------------------------------*
 * Dec_5p_5N
 *
 * Decode 5 pulses with 5*N bits:
 *-------------------------------------------------------*/
static void dec_5p_5N_fx(
    const Word32 index,   /* i  : quantization index    */
    const Word16 N,       /* i  : nb. of bits           */
    const Word16 offset,  /* i  : pulse position offset */
    Word16 pos[]    /* o  : pulse position        */
)
{
    Word16 j, n_1, tmp;
    Word32 idx;

    n_1 = sub(N, 1);
    j = add(offset, shl(1, n_1));          /* j = offset + (1 << n_1); */
    tmp = add(shl(N, 1), 1);               /* idx = (index >> ((2*N)+1)); */
    idx = L_shr(index, tmp);
    tmp = sub(extract_l(L_shr(L_mult(5, N), 1)), 1);    /* ((5*N)-1)) */

    logic16();
    IF ((L_shr(index, tmp) & 1) == 0)      /* ((index >> ((5*N)-1)) & 1)  */
    {
        dec_3p_3N1_fx(idx, n_1, offset, pos);
        dec_2p_2N1_fx(index, N, offset, pos + 3);
        move16();
    }
    ELSE
    {
        dec_3p_3N1_fx(idx, n_1, j, pos);
        dec_2p_2N1_fx(index, N, offset, pos + 3);
        move16();
    }
}

/*-------------------------------------------------------*
 * Dec_6p_6N_2
 *
 * Decode 6 pulses with 6*N+2 bits:
 *-------------------------------------------------------*/
static void dec_6p_6N2_fx(
    const Word32 index,   /* i  : quantization index    */
    const Word16 N,       /* i  : nb. of bits           */
    const Word16 offset,  /* i  : pulse position offset */
    Word16 pos[]    /* o  : pulse position        */
)
{
    Word16 j, n_1, offsetA, offsetB, n_6;

    n_1 = sub(N, 1);
    j = add(offset, shl(1, n_1));          /* j = offset + (1 << n_1); */
    n_6 = extract_l(L_mult0(N, 6));

    /* !!  N and n_1 are constants -> it doesn't need to be operated by Basic Operators */

    offsetA = offsetB = j;
    move16();
    logic16();
    IF ((L_shr(index, sub(n_6, 5)) & 1L) == 0)
    {
        /* IF (((index >> ((6*N)-5)) & 1) == 0) */
        offsetA = offset;
        move16();
    }
    ELSE
    {
        offsetB = offset;
        move16();
    }


    logic16();
    SWITCH (L_shr(index, sub(n_6,  4)) & 3)
    {
        /* (index >> ((6*N)-4)) & 3 */
    case 0:
        dec_5p_5N_fx(L_shr(index, N), n_1, offsetA, pos);  /* Dec_5p_5N(index>>N, n_1, offsetA, pos); */
        dec_1p_N1_fx(index, n_1, offsetA, pos + 5);
        move16();
        BREAK;
    case 1:
        dec_5p_5N_fx(L_shr(index, N), n_1, offsetA, pos);  /* Dec_5p_5N(index>>N, n_1, offsetA, pos); */
        dec_1p_N1_fx(index, n_1, offsetB, pos + 5);
        move16();
        BREAK;
    case 2:
        dec_4p_4N_fx(L_shr(index, add(shl(n_1,1),1)), n_1, offsetA, pos); /* Dec_4p_4N(index>>((2*n_1)+1 ), n_1, offsetA, pos); */
        dec_2p_2N1_fx(index, n_1, offsetB, pos + 4);
        move16();
        BREAK;
    case 3:
        dec_3p_3N1_fx(L_shr(index, add(add(shl(n_1, 1),n_1), 1)), n_1, offset, pos); /* Dec_3p_3N1(index>>((3*n_1)+ 1), n_1, offset, pos); */
        dec_3p_3N1_fx(index, n_1, j, pos + 3);
        move16();
        BREAK;
    }
}
static void fcb_decode_class_fx(
    Word32 index,                 /* i:   fcb index information     */
    Word32 buffer[],              /* o:   vector class               */
    Word16 pulse_num,             /* i:   number of pulses          */
    Word16 pos_num                 /* i:   number of positons        */
)
{
    Word32  i,k,l;
    Word16  temp1,temp2,temp3;
    k = L_add(index, 1);
    l = L_deposit_l(0);

    temp1 = sub(add(pos_num, pulse_num),  1);
    temp2 = add(pos_num, pulse_num);
    temp3 = pulse_num;
    move16();

    FOR (i=0; i<pulse_num-1; i++)
    {
        k = L_sub(PI_select_table_fx[extract_l(L_sub(temp1, l))][temp3], k) ;
        WHILE ( L_sub(k,PI_select_table_fx[extract_l(L_sub(temp1, l))][temp3]) < 0)
        {
            l = L_add(l,1);
        }

        k = L_sub(PI_select_table_fx[extract_l(L_sub(temp2, l))][temp3], k);

        buffer[i] = L_sub(l,1);
        move32();
        l = L_sub(l,1) ;
        temp1 = sub(temp1,1);
        temp2 = sub(temp2,1);
        temp3 = sub(temp3,1);
    }

    buffer[i] = L_sub(L_add(l, k), 1);
    move32();

    return;
}

static Word32 fcb_decode_class_all_p_fx(   /* o:   The index of pulse positions                          */
    Word32 *code_index,                 /* i:   fcb index information                              */
    Word16 sector_6p_num[],           /* o:   Number of pulses for each position                 */
    Word16 pulse_num,                   /* i:   Number of pulses on a track.                       */
    Word16 *pos_num                     /* o:   Number of positions which have pulses on the track.*/
)
{
    Word16 i;
    Word32 mn9,j,k;
    Word16 pulse_pos_num;
    Word32 vector_class_tmp[6];
    Word32 L_tmp, L_tmp1;
    FOR (i=1; i<=pulse_num; i++)
    {
        IF (L_sub((*code_index),PI_offset_fx[pulse_num][i]) < 0)
        {
            BREAK;
        }
    }

    /* (*code_index) -= PI_offset_fx[pulse_num][i-1];*/
    (*code_index) = L_sub((*code_index),PI_offset_fx[pulse_num][i-1]);

    /*pulse_pos_num = pulse_num - i + 2;*/
    pulse_pos_num = add(sub(pulse_num , i) , 2);

    /*  j = (*code_index)>>pulse_pos_num;*/
    j = L_shr((*code_index) , pulse_pos_num);
    IF(L_sub(j,PI_select_table_fx[16][pulse_pos_num]) < 0)
    {
        k = L_deposit_l(0);
        mn9 = L_add(j, 0);
    }
    ELSE
    {
        L_tmp = L_deposit_l(0);
        L_tmp1 = L_add(j, 0);
        WHILE(L_sub(L_tmp1,PI_select_table_fx[16][pulse_pos_num]) >= 0)
        {
            L_tmp = L_add(L_tmp,1);
            L_tmp1 = L_sub(L_tmp1,PI_select_table_fx[16][pulse_pos_num]);
        }
        k = L_add(L_tmp, 0);
        mn9 = L_add(L_tmp1, 0);
    }
    /* mn9 = Mult_32_32(sub(j , k) , PI_select_table_fx[16][pulse_pos_num]);*/

    test();
    IF ( (sub( pulse_pos_num,pulse_num) < 0 ) && ( sub(pulse_pos_num,1) > 0 ) )
    {
        FOR (i=0; i<pulse_pos_num; i++)
        {
            sector_6p_num[i] = 1;
            move16();
        }

        fcb_decode_class_fx(k, vector_class_tmp, pulse_num-pulse_pos_num, pulse_pos_num);

        FOR (i=0; i<pulse_num-pulse_pos_num; i++)
        {
            j = L_add(vector_class_tmp[i], 0);
            sector_6p_num[j] ++;
            move16();
        }
    }
    ELSE
    {

        IF ( sub(pulse_pos_num, 1) == 0)
        {
            sector_6p_num[0] = pulse_num;
            move16();
        }
        ELSE
        {
            FOR (i=0; i<pulse_num; i++)
            {
                sector_6p_num[i] = 1;
                move16();
            }
        }
    }

    *pos_num = pulse_pos_num;
    move16();

    return mn9;
}

static void fcb_decode_position_fx(
    Word32 index,                /* i:   position index information        */
    Word16 pos_vector[],        /* o:   the positon vector                */
    Word16 pos_num                /* i:   number of positions               */
)
{
    Word32  i,k,l;
    Word16  temp;

    k = L_add(index, 0);
    l = L_deposit_l(0);
    temp = pos_num;
    move16();
    FOR (i=0; i<pos_num-1; i++)
    {
        /* k = PI_select_table_fx[16-l][temp] - k ;*/
        k = L_sub(PI_select_table_fx[16-l][temp] , k);

        FOR (; PI_select_table_fx[16 - l][temp] >= k; l+=2);

        if (L_sub(k,PI_select_table_fx[L_sub(17,l)][temp]) > 0)
        {
            l = L_sub(l,1);
        }

        /* k = PI_select_table_fx[17-l][temp--] - k ;*/
        k = L_sub(PI_select_table_fx[L_sub(17,l)][temp--] , k);
        pos_vector[i] = extract_l(L_sub(l,1));
    }
    pos_vector[i] = extract_l(L_add(l,k));

    return;
}

static Word32 fcb_decode_PI_fx(  /* o:   return pulse position number   */
    Word32 code_index,        /* i:   fcb index information          */
    Word16 sector_6p[],     /* o:   decoded pulse position         */
    Word16 pulse_num          /* i:   pulse number for the track     */
)
{
    Word16 i,l, j;
    Word32 mn9;
    Word16 pulse_pos_num;
    Word16 sector_6p_temp[7], sector_6p_num_temp[7];
    Word16 *sector_6p_ptr0;
    Word16 *sector_6p_ptr1;

    /*get the position number and the pulse number on every position */
    mn9 = fcb_decode_class_all_p_fx(&code_index, sector_6p_num_temp, pulse_num, &pulse_pos_num);

    /* rebuild the vector*/
    /* decode the pulse position not same to the others*/
    fcb_decode_position_fx(mn9, sector_6p_temp, pulse_pos_num);
    FOR (i=pulse_pos_num-1; i>=0; i--)
    {
        /*sector_6p_temp[i] +=  ((code_index&0x1)<<4) ;*/
        sector_6p_temp[i] = add(sector_6p_temp[i],extract_l(L_shl((code_index&0x1),4))) ;
        move16();
        /*code_index = code_index>>1;*/
        code_index = L_shr(code_index , 1);

    }

    /* decode the pulse position maybe some pulse position same to other pulse */
    sector_6p_ptr0 = &sector_6p[pulse_num];
    sector_6p_ptr1 = &sector_6p_temp[pulse_pos_num];
    FOR (i=0; i<pulse_pos_num; i++)
    {
        sector_6p_ptr1 -- ;
        j = sub(pulse_pos_num, add(1,i));
        FOR (l=0; l<sector_6p_num_temp[j]; l++)
        {
            *--sector_6p_ptr0 = *sector_6p_ptr1;
            move16();
        }
    }

    return pulse_pos_num;
}



/*---------------------------------------------------------------------*
* Read FCB index                                                      *
*---------------------------------------------------------------------*/

void D_ACELP_decode_43bit(UWord16 idxs[], Word16 code[], Word16 *pulsestrack)
{
    Word32 ps[8];
    Word16 pos[7];
    Word32 joint_index;
    Word32 joint_offset = 3611648;        /*offset for 3 pulses per track*/

    set16_fx( code, 0, L_SUBFR );

    ps[3] = L_deposit_l(s_and(idxs[0], 0x1ff));
    ps[2] = L_add(L_shl(s_and(idxs[1], 3), 7), L_shr(idxs[0], 9));
    joint_index = L_shr(L_add(L_shl((Word32) idxs[2], 16), (Word32) idxs[1]), 2);

    if (L_sub(joint_index, joint_offset) >= 0)
    {
        joint_index = L_sub(joint_index, joint_offset);
    }

    iDiv_and_mod_32(joint_index, 5472, &ps[0], &ps[1], 0);
    fcb_decode_PI_fx(ps[0], pos, 3);
    add_pulses_fx(pos, pulsestrack[0], 0, code);
    fcb_decode_PI_fx(ps[1], pos, 3);
    add_pulses_fx(pos, pulsestrack[1], 1, code);

    dec_2p_2N1_fx(ps[2], 4, 0, pos);
    add_pulses_fx(pos, pulsestrack[2], 2, code);
    dec_2p_2N1_fx(ps[3], 4, 0, pos);
    add_pulses_fx(pos, pulsestrack[3], 3, code);

    return;
}

