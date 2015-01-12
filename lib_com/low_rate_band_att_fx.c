/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"
#include "stl.h"        /* required for wmc_tool */


/*--------------------------------------------------------------------------*
 * fine_gain_pred()
 *
 * Fine gain prediction
 *--------------------------------------------------------------------------*/

void fine_gain_pred_fx(
    const Word16 *sfm_start,                /* i  : Sub band start indices          */
    const Word16 *sfm_end,                  /* i  : Sub band end indices            */
    const Word16 *sfm_size,                 /* i  : Sub band bandwidths             */
    const Word16 *i_sort,                   /* i  : Energy sorting indices          */
    const Word16 *K,                        /* i  : Number of pulses per band       */
    const Word16 *maxpulse,                 /* i  : Maximum pulse per band          */
    const Word16 *R,                        /* i  : Bits per sub band           Q3  */
    const Word16 num_sfm,                   /* i  : Number of sub bands             */
    Word16 *xq,                       /* i/o: Quantized vector /quantized vector with finegain adj Q15*/
    Word16 *y,                        /* i/o: Quantized vector (int)          */
    Word16 *fg_pred,                  /* o  : Predicted fine gains        Q12 */
    const Word16 core                       /* i  : Core                            */
)
{
    Word16 i, band;
    Word16 gp;
    Word32 xx;
    Word16 accuracy;
    Word16 k, bw;

    Word16 shift, bw_idx;
    Word16 tmp, exp, exp2;
    Word32 L_tmp;
    UWord16 lsb;

    FOR( band = 0; band < num_sfm; band++)
    {
        k  = K[i_sort[band]];
        move16();

        IF( k > 0)
        {
            /*  bw, bw_idx  only used if  k>0   */
            bw     = sfm_size[i_sort[band]];
            move16();      /* allowed. 8, 16, 24,32,48,64,80,96 */
            bw_idx =  band_len_idx[ shr(bw,3) ];
            move16();      /* bw_idx=  0:                     7 */
            xx = L_deposit_l(0);
            shift = band_len_ener_shift[bw_idx];
            FOR(i = sfm_start[i_sort[band]]; i < sfm_end[i_sort[band]]; i++)
            {
                /*xx += xq[i] * xq[i]; */
                tmp = shr(xq[i], shift);    /*15-shift */
                xx = L_mac0(xx, tmp, tmp);   /*30-2*shift */
            }

            IF ( xx > 0)
            {
                /* Normalize synthesis to RMS=1.0 */
                /*gp = (float) sqrt(bw / xx); */
                exp = norm_l(xx);
                L_tmp = L_shl(xx, exp);   /*2*(15-shift)+exp */
                exp = sub(31, add(exp, sub(30, shl(shift,1))));
                L_tmp = Isqrt_lc(L_tmp, &exp);  /*31 - exp */
                Mpy_32_16_ss(L_tmp, fine_gain_pred_sqrt_bw[bw_idx], &L_tmp, &lsb);  /*31-exp+11-15=27-exp */
                gp = round_fx(L_shl(L_tmp, add(1, exp)));   /*27-exp+1+exp-16=12 */

                test();
                test();
                IF (sub(core, HQ_CORE) == 0 && R != NULL && sub(R[i_sort[band]], 256) <= 0)     /* 256 is 32 in Q3 */
                {
                    /*accuracy = ((float)k/(float)bw)*maxpulse[i_sort[band]]; */
                    L_tmp = L_mult(k, inv_tbl_fx[bw]);     /*0+15+1 */
                    exp2 = norm_l(L_tmp);
                    tmp = round_fx(L_shl(L_tmp, exp2));     /*16+exp2-16 */
                    L_tmp = L_mult0(maxpulse[i_sort[band]], tmp); /*0+exp2 */
                    exp = norm_l(L_tmp);
                    accuracy = round_fx(L_shl(L_tmp, exp)); /*exp2+exp-16=exp-16 */
                    exp = add(exp, exp2);

                    /*gp *= 1.0f - 0.05f / accuracy; */
                    tmp = div_s(13107, accuracy);   /* 0.05 in Q18 */
                    tmp = shr(tmp, sub(34, exp));   /*15+18-exp+16-15=34-exp */
                    tmp = sub(32767, tmp);
                    gp = mult_r(tmp, gp); /*15+12+1-16=12 */
                }

                fg_pred[band] = gp;
                move16();
            }
            ELSE
            {
                fg_pred[band] = 0;
                move16();
            }
        }
        ELSE
        {
            fg_pred[band] = 0;
            move16();
            FOR(i = sfm_start[i_sort[band]]; i < sfm_end[i_sort[band]]; i++)
            {
                y[i] = 0;
                move16();
                xq[i] = 0;
                move16();
            }
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * get_max_pulses()
 *
 * Find the maximum pulse height (in unit pulses) in each band
 *--------------------------------------------------------------------------*/

void get_max_pulses_fx(
    const Word16 *band_start,               /* i  : Sub band start indices    */
    const Word16 *band_end,                 /* i  : Sub band end indices      */
    const Word16 *k_sort,                   /* i  : Indices for sorting by energy */
    const Word16 *npulses,                  /* i  : Pulses per sub band       */
    const Word16  BANDS,                    /* i  : Number of bands           */
    Word16 *inp_vector,               /* i/o: Encoded shape vectors (int)*/
    Word16 *maxpulse                  /* o  : Maximum pulse height per band */
)
{
    Word16 i, k;
    Word16 npul;
    Word16 maxp;
    Word16 tmp;

    FOR (k = 0; k < BANDS; k++)
    {
        npul = npulses[k_sort[k]];
        move16();
        maxp = 0;
        move16();
        IF (npul > 0)
        {
            FOR (i = band_start[k_sort[k]]; i < band_end[k_sort[k]]; i++)
            {
                tmp = abs_s(inp_vector[i]);
                if (sub(tmp, maxp) > 0)
                {
                    maxp = tmp;
                    move16();
                }
            }
        }
        maxpulse[k_sort[k]] = maxp;
        move16();
    }

    return;
}

/*--------------------------------------------------------------------------*
 * fine_gain_dec()
 *
 * Fine gain decoder. Decodes fine gain adjustments and applies correction to
 * predicted fine gains
 *--------------------------------------------------------------------------*/

void fine_gain_dec_fx
(
    Decoder_State_fx *st,
    const Word16 *ord,          /* i  : Indices for energy order             */
    const Word16 num_sfm,       /* i  : Number of bands                      */
    const Word16 *gain_bits,    /* i  : Gain adjustment bits per sub band    */
    Word16 *fg_pred       /* i/o: Predicted gains / Corrected gains    */
)
{
    Word16 band;
    Word16 gbits;
    Word16 idx, tmp1, exp1;
    Word16 gain_dbq;
    Word32 L_tmp;


    FOR ( band = 0; band < num_sfm; band++)
    {
        gbits = gain_bits[ord[band]];
        IF (gbits > 0)
        {
            IF (fg_pred[band] != 0)
            {
                idx = get_next_indice_fx( st, gbits );
                gain_dbq = finegain_fx[gbits-1][idx];

                /* Update predicted gain with quantized correction */
                L_tmp = L_mult0(gain_dbq, 21771);   /* 21771=0.05*log2(10) */   /* 14+17=31 */
                L_tmp = L_shr(L_tmp, 15);
                tmp1 = L_Extract_lc(L_tmp, &exp1);
                tmp1 = abs_s(tmp1);
                tmp1 = extract_l(Pow2(14, tmp1));
                exp1 = sub(14, exp1);

                L_tmp = L_mult0(fg_pred[band], tmp1);   /*12+exp1 */
                fg_pred[band] = round_fx(L_shl(L_tmp, sub(16, exp1))); /*12+exp1+16-exp1-16=12 */
            }
        }
    }

    return;
}


