/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"

/*------------------------------------------------------------------
 *
 * This is an implementation of decimation-in-time FFT algorithm for
 * real sequences.  The techniques used here can be found in several
 * books, e.g., i) Proakis and Manolakis, "Digital Signal Processing",
 * 2nd Edition, Chapter 9, and ii) W.H. Press et. al., "Numerical
 * Recipes in C", 2nd Edition, Chapter 12.
 *
 * Input -  There are two inputs to this function:
 *
 *       1) An integer pointer to the input data array
 *       2) An integer value which should be set as +1 for FFT
 *          and some other value, e.g., -1 for ifFT
 *
 * Output - There is no return value.
 *       The input data are replaced with transformed data.  if the
 *       input is a real time domain sequence, it is replaced with
 *       the complex FFT for positive frequencies.  The FFT value
 *       for DC and the foldover frequency are combined to form the
 *       first complex number in the array.  The remaining complex
 *       numbers correspond to increasing frequencies.  if the input
 *       is a complex frequency domain sequence arranged as above,
 *       it is replaced with the corresponding time domain sequence.
 *
 * Notes:
 *
 *       1) This function is designed to be a part of a noise supp-
 *          ression algorithm that requires 128-point FFT of real
 *          sequences.  This is achieved here through a 64-point
 *          complex FFT.  Consequently, the FFT size information is
 *          not transmitted explicitly.  However, some flexibility
 *          is provided in the function to change the size of the
 *          FFT by specifying the size information through "define"
 *          statements.
 *
 *       2) The values of the complex sinusoids used in the FFT
 *          algorithm are computed once (i.e., the first time the
 *          r_fft function is called) and stored in a table. To
 *          further speed up the algorithm, these values can be
 *          precomputed and stored in a ROM table in actual DSP
 *          based implementations.
 *
 *       3) In the c_fft function, the FFT values are divided by
 *          2 after each stage of computation thus dividing the
 *          final FFT values by 64.  No multiplying factor is used
 *          for the ifFT.  This is somewhat different from the usual
 *          definition of FFT where the factor 1/N, i.e., 1/64, is
 *          used for the ifFT and not the FFT.  No factor is used in
 *          the r_fft function.
 *
 *       4) Much of the code for the FFT and ifFT parts in r_fft
 *          and c_fft functions are similar and can be combined.
 *          They are, however, kept separate here to speed up the
 *          execution.
 *------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*
 * c_fft_fx:
 *
 * Computes the complex part of the split-radix FFT
 *------------------------------------------------------------------------*/

static void c_fft_fx(
    const Word16 *phs_tbl,     /* i  : Table of phases            */
    Word16 SIZE,         /* i  : Size of the FFT           */
    Word16 NUM_STAGE,    /* i  : Number of stages           */
    const Word16 *in_ptr,      /* i  : coefficients in the order re[0], re[n/2], re[1], im[1], ..., re[n/2-1], im[n/2-1] */
    Word16 *out_ptr,     /* o  : coefficients in the order re[0], re[n/2], re[1], im[1], ..., re[n/2-1], im[n/2-1] */
    /* in_ptr & out_ptr must not overlap! */
    const Word16 isign)        /* i  : 1=fft, otherwise it is ifft*/
{
    Word16 i, j, k, ii, jj, kk, ji, kj;
    Word32 L_tmp1, L_tmp2;
    Word16 tmp1,tmp2,tmp3,tmp4;
    const Word16 *table_ptr;
    const Word16 *input_ptr1,*input_ptr2,*input_ptr3,*input_ptr4;

    /* Setup Reorder Variables */
    table_ptr = NULL;
    SWITCH (SIZE)
    {
    case 1024:
        table_ptr = FFT_REORDER_1024;
        BREAK;
    case 512:
        table_ptr = FFT_REORDER_512;
        BREAK;
    case 256:
        table_ptr = FFT_reorder_256;
        BREAK;
    case 128:
        table_ptr = FFT_REORDER_128;
        BREAK;
    case 64:
        table_ptr = FFT_reorder_64;
        BREAK;
    }
    /* The FFT part */
    IF (isign != 0)
    {
        /* Unrolled 1st/2nd Stage
         * 1) to take advantage of Table Values (0 & +/- 16384)
         * 2) to perform reordering of Input Values
         */
        FOR (k = 0; k < SIZE; k += 8)
        {
            /*
             * This loop use:
             *   4 Word16 (tmp1...tmp4)
             *   2 Word32 (L_tmp1 & L_tmp2)
             *   4 Pointers (table_ptr, input_ptr1, input_ptr2, input_ptr3)
             *
             * The addition of 'in_ptr' + and index value from 'reorder_ptr'
             * is counted as a move16()
             */

            input_ptr1 = in_ptr + *table_ptr++;

            L_tmp1 = L_mult(*input_ptr1++, 16384);
            L_tmp2 = L_mult(*input_ptr1, 16384);

            input_ptr1 = in_ptr + *table_ptr++;

            tmp1 = msu_r(L_tmp1, *input_ptr1, 16384);
            tmp3 = mac_r(L_tmp1, *input_ptr1++, 16384);

            input_ptr2 = in_ptr + *table_ptr++;
            input_ptr3 = in_ptr + *table_ptr++;

            L_tmp1 = L_mult(*input_ptr2++, 16384);
            tmp2 = mac_r(L_tmp1, *input_ptr3, 16384);
            tmp4 = msu_r(L_tmp1, *input_ptr3++, 16384);

            L_tmp1 = L_mult(tmp3, 16384);
            out_ptr[k] = mac_r(L_tmp1, tmp2, 16384);
            move16();
            out_ptr[k+4] = msu_r(L_tmp1, tmp2, 16384);
            move16();

            tmp2 = mac_r(L_tmp2, *input_ptr1, 16384);
            tmp3 = msu_r(L_tmp2, *input_ptr1, 16384);

            L_tmp2 = L_mult(*input_ptr2, 16384);

            L_tmp1 = L_mult(tmp1, 16384);
            tmp1 = msu_r(L_tmp2, *input_ptr3, 16384);
            out_ptr[k+2] = mac_r(L_tmp1, tmp1, 16384);
            move16();
            out_ptr[k+6] = msu_r(L_tmp1, tmp1, 16384);
            move16();

            L_tmp1 = L_mult(tmp2, 16384);
            tmp2 = mac_r(L_tmp2, *input_ptr3, 16384);
            out_ptr[k+1] = mac_r(L_tmp1, tmp2, 16384);
            move16();
            out_ptr[k+5] = msu_r(L_tmp1, tmp2, 16384);
            move16();

            L_tmp1 = L_mult(tmp3, 16384);
            out_ptr[k+3] = msu_r(L_tmp1, tmp4, 16384);
            move16();
            out_ptr[k+7] = mac_r(L_tmp1, tmp4, 16384);
            move16();
        }

        /* Remaining Stages */
        FOR (i = 2; i < NUM_STAGE; i++)
        {
            /* i is stage counter      */
            jj = shl(2, i);             /* FFT size                */
            kk = shl(jj, 1);            /* 2 * FFT size            */
            ii = shr(SIZE, i);
            ji = 0;
            move16();       /* ji is phase table index */

            FOR (j = 0; j < jj; j += 2)
            {
                /* j is sample counter     */
                FOR (k = j; k < SIZE; k += kk)
                {
                    /* k is butterfly top     */
                    kj = add(k, jj);              /* kj is butterfly bottom */

                    /* Butterfly computations */
                    L_tmp1 = L_msu(L_mult(*(out_ptr + kj), phs_tbl[ji]),
                                   *(out_ptr + kj + 1), phs_tbl[ji + 1]);
                    L_tmp2 = L_mac(L_mult(*(out_ptr + kj + 1), phs_tbl[ji]),
                                   *(out_ptr + kj), phs_tbl[ji + 1]);

                    out_ptr[kj] = mac_r(L_negate(L_tmp1), out_ptr[k], 16384);
                    move16();
                    out_ptr[kj+1] = mac_r(L_negate(L_tmp2), out_ptr[k+1], 16384);
                    move16();
                    out_ptr[k] = mac_r(L_tmp1, out_ptr[k], 16384);
                    move16();
                    out_ptr[k+1] = mac_r(L_tmp2, out_ptr[k+1], 16384);
                    move16();
                }
                ji = add(ji, ii);
            }
        }
    }
    ELSE /* The ifFT part */
    {
        /* Unrolled 1st/2nd Stage
         * 1) to take advantage of Table Values (0 & +/- 16384)
         * 2) to perform reordering of Input Values
         */
        FOR (k = 0; k < SIZE; k += 8)
        {
            /*
             * This loop use:
             *   4 Word16 (tmp1...tmp4)
             *   2 Word32 (L_tmp1 & L_tmp2)
             *   5 Pointers (reorder_ptr, input_ptr1...input_ptr4)
             *
             * The addition of 'in_ptr' + and index value from 'reorder_ptr'
             * is counted as a move16()
             */

            input_ptr1 = in_ptr + *table_ptr++;
            input_ptr2 = in_ptr + *table_ptr++;

            input_ptr3 = in_ptr + *table_ptr++;
            input_ptr4 = in_ptr + *table_ptr++;

            tmp3 = sub(*input_ptr1, *input_ptr2);
            tmp4 = add(*input_ptr1++, *input_ptr2++);

            tmp2 = sub(input_ptr3[0], input_ptr4[0]);
            tmp1 = sub(input_ptr3[1], input_ptr4[1]);

            out_ptr[k+2] = sub(tmp3, tmp1);
            move16();
            out_ptr[k+6] = add(tmp3, tmp1);
            move16();

            tmp1 = sub(*input_ptr1, *input_ptr2);
            out_ptr[k+3] = add(tmp1, tmp2);
            move16();
            out_ptr[k+7] = sub(tmp1, tmp2);
            move16();

            tmp1 = add(input_ptr3[0], input_ptr4[0]);
            tmp3 = add(input_ptr3[1], input_ptr4[1]);

            out_ptr[k] = add(tmp4, tmp1);
            move16();
            out_ptr[k+4] = sub(tmp4, tmp1);
            move16();

            tmp4 = add(*input_ptr1, *input_ptr2);
            out_ptr[k+1] = add(tmp4, tmp3);
            move16();
            out_ptr[k+5] = sub(tmp4, tmp3);
            move16();
        }

        table_ptr = phs_tbl + SIZE;  /* access part of table that is scaled by 2 */

        /* Remaining Stages */
        FOR (i = 2; i < NUM_STAGE; i++)
        {
            /* i is stage counter      */
            jj = shl(2, i);             /* FFT size                */
            kk = shl(jj, 1);            /* 2 * FFT size            */
            ii = shr(SIZE, i);
            ji = 0;
            move16();     /* ji is phase table index */

            FOR (j = 0; j < jj; j += 2)
            {
                /* j is sample counter     */
                /* This can be computed by successive add_fxitions of ii to ji, starting from 0
                   hence line-count it as a one-line add (still need to increment op count!!) */

                FOR (k = j; k < SIZE; k += kk)
                {
                    /* k is butterfly top     */
                    kj = add(k, jj);            /* kj is butterfly bottom */

                    /* Butterfly computations */
                    tmp1 = mac_r(L_mult(out_ptr[kj], table_ptr[ji]),
                    out_ptr[kj+1], table_ptr[ji + 1]);

                    tmp2 = msu_r(L_mult(out_ptr[kj+1], table_ptr[ji]),
                    out_ptr[kj], table_ptr[ji+1]);

                    out_ptr[kj] = sub(out_ptr[k], tmp1);
                    move16();
                    out_ptr[kj+1] = sub(out_ptr[k+1], tmp2);
                    move16();
                    out_ptr[k] = add(out_ptr[k], tmp1);
                    move16();
                    out_ptr[k+1] = add(out_ptr[k+1], tmp2);
                    move16();
                }
                ji = add(ji, ii);
            }
        }
    }
}

/*--------------------------------------------------------------------------------*
 * r_fft_fx:
 *
 * Perform FFT fixed-point for real-valued sequences of length 32, 64 or 128
 *--------------------------------------------------------------------------------*/
void r_fft_fx_lc(
    const Word16 *phs_tbl,    /* i  : Table of phase            */
    const Word16 SIZE,        /* i  : Size of the FFT           */
    const Word16 SIZE2,       /* i  : Size / 2                  */
    const Word16 NUM_STAGE,   /* i  : Number of stage           */
    const Word16 *in_ptr,     /* i  : coeffients in the order re[0], re[1], ... re[n/2], im[1], im[2], ..., im[n/2-1] */
    Word16 *out_ptr,    /* o  : coeffients in the order re[0], re[1], ... re[n/2], im[1], im[2], ..., im[n/2-1] */
    const Word16 isign        /* i  : 1=fft, otherwize it's ifft                                                      */
)
{
    Word16 tmp2_real, tmp2_imag;
    Word32 Ltmp1_real, Ltmp1_imag;
    Word16 i;
    Word32 Ltmp1;
    const Word16 *phstbl_ptrDn;
    Word16 *ptrDn;
    Word16 temp[1024];  /* Accommodates real input FFT size up to 1024. */

    /* Setup Pointers */
    phstbl_ptrDn = &phs_tbl[SIZE-1];

    /* The FFT part */
    IF (isign != 0)
    {
        Word16 *ptRealUp, *ptRealDn, *ptImaUp, *ptImaDn;

        /* Perform the complex FFT */
        c_fft_fx(phs_tbl, SIZE, NUM_STAGE, in_ptr, temp, isign);

        /* First, handle the DC and foldover frequencies */
        out_ptr[SIZE2] = sub(temp[0], temp[1]);
        move16();
        out_ptr[0] = sub(add(temp[0], temp[1]), shr(NUM_STAGE, 1));
        move16();/* DC have a small offset */

        ptrDn = &temp[SIZE-1];

        ptImaDn = &out_ptr[SIZE-1];
        ptRealUp = &out_ptr[1];
        ptImaUp = &out_ptr[SIZE2+1];
        ptRealDn = &out_ptr[SIZE2-1];

        /* Now, handle the remaining positive frequencies */
        FOR (i = 2; i <= SIZE2; i += 2)
        {
            Ltmp1_imag = L_mult(temp[i+1], 16384);
            Ltmp1_imag = L_msu(Ltmp1_imag, *ptrDn, 16384);
            tmp2_real = add(temp[i+1], *ptrDn--);

            Ltmp1_real = L_mult(temp[i], 16384);
            Ltmp1_real = L_mac(Ltmp1_real, *ptrDn, 16384);
            tmp2_imag = sub(*ptrDn--, temp[i]);


            *ptRealUp++ = msu_r(L_mac(Ltmp1_real, tmp2_real, phs_tbl[i]), tmp2_imag, phs_tbl[i+1]);
            move16();
            *ptImaDn-- = mac_r(L_mac(Ltmp1_imag, tmp2_imag, phs_tbl[i]), tmp2_real, phs_tbl[i+1]);
            move16();
            Ltmp1 = L_mac(L_negate(Ltmp1_imag), tmp2_real, *phstbl_ptrDn);
            Ltmp1_real = L_mac(Ltmp1_real, tmp2_imag, *phstbl_ptrDn--);
            *ptImaUp++ = msu_r(Ltmp1, tmp2_imag, *phstbl_ptrDn);
            move16();
            *ptRealDn-- = mac_r(Ltmp1_real, tmp2_real, *phstbl_ptrDn--);
            move16();
        }
    }
    ELSE /* The ifFT part */
    {
        const Word16 *ptRealUp, *ptRealDn, *ptImaUp, *ptImaDn;

        /* First, handle the DC and foldover frequencies */
        Ltmp1 = L_mult(in_ptr[0], 16384);
        temp[0] = mac_r(Ltmp1, in_ptr[SIZE2], 16384);
        move16();
        temp[1] = msu_r(Ltmp1, in_ptr[SIZE2], 16384);
        move16();

        ptrDn = &temp[SIZE-1];

        /* Here we cast to Word16 * from a const Word16 *. */
        /* This is ok because we use these pointers for    */
        /* reading only. This is just to avoid declaring a */
        /* bunch of 4 other pointer with const Word16 *.   */
        ptImaDn = &in_ptr[SIZE-1];
        ptRealUp = &in_ptr[1];
        ptImaUp = &in_ptr[SIZE2+1];
        ptRealDn = &in_ptr[SIZE2-1];

        /* Now, handle the remaining positive frequencies */
        FOR (i = 2; i <= SIZE2; i += 2)
        {
            Ltmp1_imag = L_mult(*ptImaDn, 16384);
            Ltmp1_imag = L_msu(Ltmp1_imag, *ptImaUp, 16384);
            tmp2_real = add(*ptImaDn--, *ptImaUp++);
            Ltmp1_real = L_mult(*ptRealUp, 16384);
            Ltmp1_real = L_mac(Ltmp1_real, *ptRealDn, 16384);
            tmp2_imag = sub(*ptRealUp++, *ptRealDn--);


            temp[i] = mac_r(L_msu(Ltmp1_real, tmp2_real, phs_tbl[i]), tmp2_imag, phs_tbl[i+1]);
            move16();
            temp[i+1] = mac_r(L_mac(Ltmp1_imag, tmp2_imag, phs_tbl[i]), tmp2_real, phs_tbl[i+1]);
            move16();
            Ltmp1 = L_mac(L_negate(Ltmp1_imag), tmp2_real, *phstbl_ptrDn);
            Ltmp1_real = L_msu(Ltmp1_real, tmp2_imag, *phstbl_ptrDn--);
            *ptrDn-- = msu_r(Ltmp1, tmp2_imag, *phstbl_ptrDn);
            move16();
            *ptrDn-- = msu_r(Ltmp1_real, tmp2_real, *phstbl_ptrDn--);
            move16();
        }

        /* Perform the complex ifFT */
        c_fft_fx(phs_tbl, SIZE, NUM_STAGE, temp, out_ptr, isign);
    }
}
