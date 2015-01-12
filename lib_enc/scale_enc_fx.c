/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * Scale_wsp
 *
 * Find scaling factor for weighted speech input
 *-------------------------------------------------------------------*/
void Scale_wsp(
    Word16 *wsp,                /* i  : Weigthed speech                      */
    Word16 *old_wsp_max,        /* i  : Last weigthed speech maximal valu    */
    Word16 *shift,              /* i/o: Scaling of current frame             */
    Word16 *Q_exp,              /* i/o: Differential scaling factor          */
    Word16 *old_wsp_shift,      /* i/o: Last wsp scaling                     */
    Word16 *old_wsp,            /* i/o: Old weighted speech buffer           */
    Word16 *mem_decim2,         /* i/o: Decimation buffer                    */
    Word16 *old_wsp12k8,        /* i/o: wsp memory @ 12.8 kHz used in pitol2 */
    const  Word16 Len_p_look           /* i  : L_frame + look ahead                 */
)
{
    Word16 max, i, tmp;

    /* find maximum value on wsp[] for 12 bits scaling */
    max = 0;
    move16();
    FOR (i = 0; i < Len_p_look; i++)
    {
        tmp = abs_s(wsp[i]);
        max = s_max(max,tmp);
    }
    tmp = *old_wsp_max;
    move16();
    tmp = s_max(max, tmp);
    *old_wsp_max = max;
    move16();

    *shift = sub(norm_s(tmp), 3);
    move16();

    *shift = s_min(*shift, 0);
    move16(); /* shift = 0..-3 */


    Scale_sig(wsp, Len_p_look, *shift);
    /* scale old_wsp (warning: exp must be Q_new-Q_old) */
    *Q_exp= add(*Q_exp, sub(*shift, *old_wsp_shift));
    move16();
    *old_wsp_shift = *shift;
    move16();
    Scale_sig(old_wsp12k8, L_WSP_MEM, *Q_exp); /* Already scaled with premphasis */
    Scale_sig(old_wsp, (L_WSP_MEM - L_INTERPOL)/OPL_DECIM, *Q_exp );
    Scale_sig(mem_decim2, 3, *Q_exp);

    Copy( old_wsp12k8, wsp - L_WSP_MEM, L_WSP_MEM ); /* Now memory and wsp vector have the same scaling */
}

/*-------------------------------------------------------------------*
 * Preemph_scaled
 *
 * Find scaled preemphasis vector and its scaling factor
 *-------------------------------------------------------------------*/
void Preemph_scaled(
    Word16 new_speech[],     /* i  : Speech to scale already on 14 bits*/
    Word16 *Q_new,           /* o  : Scaling factor                  */
    Word16 *mem_preemph,     /* i/o: Preemph memory                  */
    Word16 *Q_max,           /* i/o: Q_new limitation                */
    const Word16 Preemph_factor,   /* i  : Preemphasis factor         Q15  */
    const Word16 bits,             /* i  : Bit to remove from the output to (15-bits)*/
    const Word16 bit1,             /* i  : Limit the output scaling to ((15-bits)-bit1) bits  */
    const Word16 L_Q_mem,          /* i  : Number of old scaling to take into account  */
    const Word16 Lframe,           /* i  : Frame length                    */
    const Word16 last_coder_type,  /* i  : coder_type                      */
    const Word16 Search_scaling    /* i  : enable the search of a proper scaling factor*/
)
{
    Word16 i, tmp_fixed;
    Word16 mu, shift, QVal;
    Word32 L_tmp, L_maxloc;
    Word16 Q_min;

    /*---------------------------------------------------------------*
     * Perform fixed preemphasis through 1 - g z^-1 *
     * Scale signal to get maximum of precision in filtering *
     *---------------------------------------------------------------*/

    BASOP_SATURATE_WARNING_OFF
    QVal = shl(1, sub(15,bits));
    BASOP_SATURATE_WARNING_ON
    mu = shr(Preemph_factor, bits); /* Q15 --> Q(15-bits) */

    IF(sub(Search_scaling,1)==0)
    {
        /* get max of new preemphased samples (L_FRAME+L_FILT) */

        L_tmp = L_mult(new_speech[0], QVal);
        L_tmp = L_msu(L_tmp, *mem_preemph, mu);
        L_maxloc = L_abs(L_tmp);

        FOR (i = 1; i < Lframe; i++)
        {
            /* Equivalent to tmp = max((abs(x[i] - mu*x[i-1]),tmp)
            * finds the max of preemphasized signal */
            L_tmp = L_mult(new_speech[i], QVal);
            L_tmp = L_msu(L_tmp, new_speech[i - 1], mu);
            L_tmp = L_abs(L_tmp);
            L_maxloc = L_max(L_tmp, L_maxloc);
        }

        /* get scaling factor for new and previous samples */
        /* limit scaling to Q_MAX to keep dynamic for ringing in low signal */
        /* limit scaling to Q_MAX also to avoid a[0]<1 in syn_filt_32 */
        tmp_fixed = s_max(extract_h(L_maxloc),1);

        /* output on 14 bits: needed unless the resampling itself removes 1 bit*/
        shift = sub(norm_s(tmp_fixed), add(bits,bit1));
        shift = s_max(shift, 0);
        shift = s_min(shift, Q_MAX);

        minimum_fx(Q_max, L_Q_mem, &Q_min);
        *Q_new = s_min(shift, Q_min);
        move16();

        IF (tmp_fixed == 0)
        {
            IF ( sub(last_coder_type, UNVOICED) != 0 )
            {
                *Q_new = s_min(*Q_new, 1);
                move16();
            }
        }

        FOR(i = L_Q_mem-1; i > 0; i--)
        {
            Q_max[i] = Q_max[i-1];
            move16();
        }
        Q_max[i] = shift;
        move16();
    }

    /*---------------------------------------------------------------*
     * preemphasis with scaling (L_FRAME+L_FILT)
     * now do the actual preemphasis, since we have the
     * proper scaling factor.
     * Done backwards to save storage space
     *---------------------------------------------------------------*/

    tmp_fixed = new_speech[Lframe - 1];
    move16();

    FOR (i = sub(Lframe,1); i > 0; i--)
    {
        L_tmp = L_mult(new_speech[i], QVal);
        L_tmp = L_msu(L_tmp, new_speech[i - 1], mu);
        L_tmp = L_shl(L_tmp, *Q_new);
        new_speech[i] = round_fx(L_tmp);
    }

    L_tmp = L_mult(new_speech[0], QVal);
    L_tmp = L_msu(L_tmp, *mem_preemph, mu);
    L_tmp = L_shl(L_tmp, *Q_new);
    new_speech[0] = round_fx(L_tmp);

    *mem_preemph = tmp_fixed;
    move16();
}
/*-------------------------------------------------------------------*
 * Scale_mem
 *
 * Rescale memories
 *-------------------------------------------------------------------*/
Word32 Scale_mem_pre_proc(                 /* o  : Min energy scaled           */
    Word16 ini_frame_fx,      /* i  : Frame number                */
    Word16 Q_exp,             /* i  : Diff scaling factor         */
    Word16 *Q_new,            /* i/o: Absolute scaling factor     */
    Word16 *old_speech,       /* i/o: Speech memory               */
    Word16 *mem_wsp,          /* i/o: wsp vector memory           */
    Word32 *enrO,             /* i/o: Enr mem                     */
    Word32 *bckr,             /* i/o: Back ground_fx ener mem     */
    Word32 *ave_enr,          /* i/o: Ave_enr mem                 */
    Word32 *ave_enr2,          /* i/o: Ave_enr2 mem                 */
    Word32 *st_fr_bands1,     /* i/o: spectrum per critical bands of the previous frame  */
    Word32 *st_fr_bands2,     /* i/o: spectrum per critical bands 2 frames ago           */
    Word32 *st_Bin_E_old
)
{
    Word16 i;
    Word32 e_min_scaled;

    e_min_scaled = L_shr_r(L_add(L_shr(E_MIN_FXQ15,sub(14,add(*Q_new,QSCALE))),1),1);

    /* scale previous samples and memory (Q_exp - Q_new - Q_old) */
    /* Scale( x, y, z ) : shift left vector x of size y by z bits ) */
    IF (Q_exp != 0)
    {
        IF(old_speech != NULL)
        {
            Scale_sig(old_speech, L_INP_MEM, Q_exp);
        }
        Scale_sig(mem_wsp, 1, Q_exp);
        IF( ini_frame_fx ==0 )
        {
            /* Scaling noise vectors if frame ==1*/
            Scale_sig32(enrO, NB_BANDS,*Q_new);
            Scale_sig32(bckr, NB_BANDS, *Q_new);
            Scale_sig32(ave_enr, NB_BANDS, *Q_new);
            Scale_sig32(ave_enr2, NB_BANDS, *Q_new);
            Scale_sig32(st_fr_bands1, NB_BANDS, *Q_new);
            Scale_sig32(st_fr_bands2, NB_BANDS, *Q_new);
            Scale_sig32(st_Bin_E_old, L_FFT/2, *Q_new);
        }
        ELSE
        {
            /* Do scaling and valide minimum energy value */
            FOR (i = 0; i < NB_BANDS; i++)
            {
                enrO[i] = L_max(L_shl(enrO[i], Q_exp),e_min_scaled);
                move32();
                bckr[i] = L_max(L_shl(bckr[i], Q_exp),e_min_scaled);
                move32();
                ave_enr[i] = L_max(L_shl(ave_enr[i], Q_exp),e_min_scaled);
                move32();
                ave_enr2[i] = L_max(L_shl(ave_enr2[i], Q_exp),e_min_scaled);
                move32();
                st_fr_bands1[i] = L_max(L_shl(st_fr_bands1[i], Q_exp),e_min_scaled);
                move32();
                st_fr_bands2[i] = L_max(L_shl(st_fr_bands2[i], Q_exp),e_min_scaled);
                move32();
            }
        }
    }
    return e_min_scaled;
}

void Scale_mem_enc(
    Word16 Q_exp,             /* i  : Diff scaling factor         */
    Word16 *old_speech16k,    /* i/o: Speech memory               */
    Word16 *old_exc,          /* i/o: excitation memory           */
    Word16 *old_bwe_exc,      /* i/o: BWE excitation memory       */
    Word16 *mem_w0,           /* i/o: target vector memory        */
    Word16 *mem_syn,          /* i/o: synthesis memory            */
    Word16 *mem_syn2,         /* i/o: synthesis memory            */
    Word16 *mem_deemp_preQ_fx, /*i/o: deemphasis memory for the high rate celp codec */
    Word16 *last_exc_dct_in,
    Word16 *old_input_lp
)
{
    /* scale previous samples and memory (Q_exp - Q_new - Q_old) */
    /* Scale( x, y, z ) : shift left vector x of size y by z bits ) */
    IF (Q_exp != 0)
    {
        Scale_sig(old_speech16k, L_INP_MEM, Q_exp);
        Scale_sig(mem_w0, 1, Q_exp);
        /* Scaling excitation */
        Scale_sig(old_exc,L_EXC_MEM, Q_exp);
        Scale_sig(old_bwe_exc, PIT16k_MAX*2, Q_exp);
        Scale_sig(mem_syn, M, Q_exp);
        Scale_sig(mem_syn2, M, Q_exp);
        Scale_sig(last_exc_dct_in,L_FRAME, Q_exp);
        Scale_sig(mem_deemp_preQ_fx, 1, Q_exp);
        Scale_sig(old_input_lp, NS2SA(16000, ACELP_LOOK_NS + DELAY_SWB_TBE_16k_NS + DELAY_FIR_RESAMPL_NS), Q_exp);
    }

    return;
}

