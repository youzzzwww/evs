/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "stl.h"
#include "basop_util.h"
#include "prot_fx.h"
#include "options.h"

#define swap(x,y,type) {type u__p; u__p=x; x=y; y=u__p;}

extern const Word16 tbl_mid_gen_wb_5b_fx[];
extern const Word16 tbl_mid_unv_wb_5b_fx[];




void midlsf_dec(
    const Word16 qlsf0[],  /* i: quantized lsf coefficients (3Q12) */
    const Word16 qlsf1[],  /* i: quantized lsf coefficients (3Q12) */
    Word16 idx,        /* i: codebook index          */
    Word16 qlsf[],      /* o: decoded lsf coefficients   (3Q12) */
    Word16 coder_type,
    Word16 *mid_lsf_int,
    Word16 prev_bfi,
    Word16 safety_net)
{
    const Word16 *ratio=NULL;
    Word16 j;
    Word32 L_tmp;
    Word16 bad_spacing = 0;

    move16();
    /* Select codebook */
    IF ( sub(coder_type, UNVOICED) == 0 )
    {
        ratio = tbl_mid_unv_wb_5b_fx;
    }
    ELSE
    {
        ratio = tbl_mid_gen_wb_5b_fx;
    }
    FOR (j=0; j<M; j++)
    {
        L_tmp = L_mult(sub(0x2000, ratio[idx*M+j]), qlsf0[j]); /*Q(x2.56+13+1)->Q(x2.56+14)*/
        L_tmp = L_mac(L_tmp, ratio[idx*M+j], qlsf1[j]); /*Q(x2.56+14)*/
        qlsf[j] = round_fx(L_shl(L_tmp,2)); /*Q(x2.56)*/
    }


    IF(mid_lsf_int != NULL) /*at the decoder*/
    {
        /* check for incorrect LSF ordering */
        IF ( sub(*mid_lsf_int, 1) == 0 )
        {
            FOR (j=1; j<M; j++)
            {
                IF ( sub(qlsf[j] , qlsf[j-1]) < 0 )
                {
                    bad_spacing = 1;
                    move16();
                    BREAK;
                }
            }
        }
        /* Redo mid-LSF interpolation with 0.4 in case of LSF instability */
        test();
        test();
        IF( prev_bfi || ( sub(*mid_lsf_int, 1) == 0 && bad_spacing ) )
        {
            FOR (j=0; j<M; j++)
            {
                /* redo mid-LSF interpolation with 0.4 */
                qlsf[j] = add(mult_r(13107, qlsf0[j]), mult_r(19661, qlsf1[j])); /* Q15 +x2.56 -Q15 13107 = 0.4(Q15), 19661 = 0.6 (Q15)*/ move16();

                /* ensure correct ordering of LSF indices */
                test();
                test();
                IF ( j > 0 && sub(j, M) <0 && sub(qlsf[j], add( qlsf[j-1], LSF_GAP_MID_FX))<0  )
                {
                    qlsf[j] = add(qlsf[j-1], LSF_GAP_MID_FX);
                    move16();
                }

            }
        }
        ELSE
        {
            /* otherwise, use regular LSF spacing and ordering as in the encoder */
            FOR (j=0; j<M; j++)
            {
                test();
                test();
                IF ( j > 0 && sub(j, M) < 0 && sub(qlsf[j], add( qlsf[j-1],LSF_GAP_MID_FX))<0 )
                {
                    qlsf[j] = add(qlsf[j-1], LSF_GAP_MID_FX);
                    move16();
                }
            }
        }
        if ( prev_bfi )
        {
            /* continue redoing mid-LSF interpolation with 0.4 in order not to propagate the error */
            *mid_lsf_int = 1;
            move16();
        }
        if ( safety_net )
        {
            /* safety-net encountered -> stop redoing mid-LSF interpolation with 0.4 */
            *mid_lsf_int = 0;
            move16();
        }
    }
    ELSE
    {
        /* use regular LSF spacing */
        FOR (j=0; j<M; j++)
        {
            test();
            test();
            IF ( j > 0 && sub(j, M) <0 && sub(qlsf[j], add(qlsf[j-1], LSF_GAP_MID_FX))<0 )
            {
                qlsf[j] = add(qlsf[j-1], LSF_GAP_MID_FX);
                move16();
            }
        }
    }

    return;
}
Word16 lsf_ind_is_active(
    const Word16 lsf_q_ind[],
    const Word16 means[],
    Word16 narrowband,
    Word16 cdk
)
{
    Word16 lsf[2], min_distance;

    lsf[0] = add(lsf_q_ind[0], means[0]);
    move16();
    lsf[1] = add(lsf_q_ind[1], means[1]);
    move16();

    min_distance = lsf[0];
    move16();
    min_distance = s_min(min_distance, sub(lsf[1], lsf[0]));

    assert(narrowband == 0 || narrowband == 1);
    assert(cdk == 0 || cdk == 1);

    return sub(min_distance, min_distance_thr[narrowband][cdk]) < 0;
}

