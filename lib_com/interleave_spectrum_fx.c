/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"          /* required for wmc_tool */

/*--------------------------------------------------------------------------*
 * interleave_spectrum_fx()
 *
 * Interleave the spectrum
 *--------------------------------------------------------------------------*/

void interleave_spectrum_fx(
    Word32 *coefs,                  /* i/o: input and output coefficients   Q12  */
    const Word16 length             /* i  : length of spectrum              Q0   */
)
{
    Word16   i, j, k;
    Word32   *p1, *p2, *p3, *p4;
    Word32   *p_out;
    Word32   coefs_out[L_FRAME48k];
    Word16   sublen[3] = {240, 160, 80};
    Word16   grps;
    const Word16   *bw;
    const Word16   *cnt;

    /* Common inits */
    p1 = coefs;
    p_out = coefs_out;

    IF ( sub(length, L_FRAME48k)  == 0 )
    {
        bw = intl_bw_48;
        cnt = intl_cnt_48;
        grps = N_INTL_GRP_48;
        move16();
        p2 = p1 + sublen[0];
        p3 = p2 + sublen[0];
        p4 = p3 + sublen[0];
    }
    ELSE IF( sub(length, L_FRAME32k) == 0 )
    {
        bw = intl_bw_32;
        cnt = intl_cnt_32;
        grps = N_INTL_GRP_32;
        move16();
        p2 = p1 + sublen[1];
        p3 = p2 + sublen[1];
        p4 = p3 + sublen[1];
    }
    ELSE /* length == L_FRAME16k */
    {
        bw = intl_bw_16;
        cnt = intl_cnt_16;
        grps = N_INTL_GRP_16;
        move16();
        p2 = p1 + sublen[2];
        p3 = p2 + sublen[2];
        p4 = p3 + sublen[2];
    }

    FOR (i = 0; i < grps; i++)
    {
        FOR (j = 0; j < cnt[i]; j++)
        {
            FOR (k = 0; k < bw[i]; k++)
            {
                *p_out++ = *p1++;
                move32();
            }
            FOR (k = 0; k < bw[i]; k++)
            {
                *p_out++ = *p2++;
                move32();
            }
            FOR (k = 0; k < bw[i]; k++)
            {
                *p_out++ = *p3++;
                move32();
            }
            FOR (k = 0; k < bw[i]; k++)
            {
                *p_out++ = *p4++;
                move32();
            }
        }
    }

    /* For FB the interleaved spectrum is 800 samples */
    Copy32(coefs_out, coefs, (Word16)(p_out - coefs_out));

    return;
}

/*--------------------------------------------------------------------------*
 * de_interleave_spectrum_fx()
 *
 * Deinterleave the spectrum
 *--------------------------------------------------------------------------*/

void de_interleave_spectrum_fx(
    Word32 *coefs,                  /* i/o: input and output coefficients   Q12 */
    const Word16 length             /* i  : length of spectrum              Q0  */
)
{
    Word16   i, j, k;
    Word32   *p1, *p2, *p3, *p4;
    Word32   *p_in;
    Word32   coefs_out[L_FRAME48k];
    Word16   sublen[] = {80, 160, 240, 320, 480, 720};
    Word16   grps;
    const Word16   *bw;
    const Word16   *cnt;

    /* common for all groups */
    p1 = coefs_out;

    IF ( sub(length, L_FRAME48k)  == 0 )
    {
        bw = intl_bw_48;
        cnt = intl_cnt_48;
        grps = N_INTL_GRP_48;
        move16();

        p2 = coefs_out + sublen[2];     /* 240, length/4 */
        p3 = coefs_out + sublen[4];     /* 480, 2*length/4 */
        p4 = coefs_out + sublen[5];     /* 720, 3*length/4 */
    }
    ELSE IF( sub(length, L_FRAME32k) == 0 )
    {
        bw = intl_bw_32;
        cnt = intl_cnt_32;
        grps = N_INTL_GRP_32;
        move16();

        p2 = coefs_out + sublen[1];     /* 160 */
        p3 = coefs_out + sublen[3];     /* 320 */
        p4 = coefs_out + sublen[4];     /* 480 */
    }
    ELSE /* length == L_FRAME16k */
    {
        bw = intl_bw_16;
        cnt = intl_cnt_16;
        grps = N_INTL_GRP_16;
        move16();

        p2 = coefs_out + sublen[0];     /* 80 */
        p3 = coefs_out + sublen[1];     /* 160 */
        p4 = coefs_out + sublen[2];     /* 240 */
    }

    set32_fx(coefs_out, 0, L_FRAME48k);
    p_in = coefs;

    FOR (i = 0; i < grps; i++)
    {
        FOR (j = 0; j < cnt[i]; j++)
        {
            FOR (k = 0; k < bw[i]; k++)
            {
                *p1++ = *p_in++;
                move32();
            }
            FOR (k = 0; k < bw[i]; k++)
            {
                *p2++ = *p_in++;
                move32();
            }
            FOR (k = 0; k < bw[i]; k++)
            {
                *p3++ = *p_in++;
                move32();
            }
            FOR (k = 0; k < bw[i]; k++)
            {
                *p4++ = *p_in++;
                move32();
            }
        }
    }

    Copy32(coefs_out, coefs, length);

    return;
}

