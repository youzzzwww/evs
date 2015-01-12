/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"            /* required for wmc_tool */


/*--------------------------------------------------------------------------
 * subband_gain_bits()
 *
 * HQ core encoder
 *--------------------------------------------------------------------------*/

void subband_gain_bits_fx(
    const Word16 *Rk,            /* i  : bit allocation per band Q3 */
    const Word16 N,              /* i  : number of bands         */
    Word16 *bits,          /* o  : gain bits per band      */
    const Word16 *sfmsize        /* i  : Size of bands           */
)
{
    Word16 i,b,tot;
    Word16 bps;

    tot = 0;
    move16();

    FOR ( i = 0; i < N; i++ )
    {
        /*bps = (short)(Rk[i]/sfmsize[i]); */
        bps = extract_l(L_shr(L_mult0(Rk[i], inv_tbl_fx[sfmsize[i]]), 18));  /*3+15 */

        if (L_sub(L_shl(L_mult0(sfmsize[i], add(bps, 1)), 3), Rk[i]) == 0)
        {
            bps = add(bps, 1);
        }

        bps = s_min(7, bps);
        b = fine_gain_bits[bps];
        move16();
        bits[i] = b;
        move16();
        tot = add(tot, b);
    }

    if ( tot == 0)
    {
        /* If no gain bits were assigned, use one bit anyway for potential PVQ overage */
        bits[0] = 1;
        move16();
    }

    return;
}

/*--------------------------------------------------------------------------*
 * assign_gain_bits()
 *
 * Assign gain adjustment bits and update bit budget
 *--------------------------------------------------------------------------*/

Word16 assign_gain_bits_fx(           /* o  : Number of assigned gain bits          */
    const Word16 core,                /* i  : HQ core                               */
    const Word16 BANDS,               /* i  : Number of bands                       */
    const Word16 *band_width,         /* i  : Sub band bandwidth                    */
    Word16 *Rk,                 /* i/o: Bit allocation/Adjusted bit alloc. Q3 */
    Word16 *gain_bits_array,    /* o  : Assigned gain bits                    */
    Word16 *Rcalc               /* o  : Bit budget for shape quantizer     Q3 */
)
{
    Word16 subband_cnt;
    Word16 gain_bits_tot;
    Word16 i;

    /* Allocate gain bits for every subband used, based on bit rate and bandwidth */
    IF( sub(core, HQ_CORE) == 0 )
    {
        subband_gain_bits_fx(Rk, BANDS, gain_bits_array, band_width);
    }
    ELSE
    {
        set16_fx( gain_bits_array, 0, BANDS );
    }

    /* Re-adjust bit budget for gain quantization */
    subband_cnt = 0;
    move16();
    gain_bits_tot = 0;
    move16();
    *Rcalc = 0;
    move16();
    FOR (i = 0; i < BANDS; i++)
    {
        IF (Rk[i] > 0)
        {
            subband_cnt = add(subband_cnt, 1);
            Rk[i] = sub(Rk[i], shl(gain_bits_array[i], 3));
            move16();
            gain_bits_tot = add(gain_bits_tot, gain_bits_array[i]);
            *Rcalc = add(*Rcalc, Rk[i]);
            move16();
        }
    }

    return gain_bits_tot;
}
