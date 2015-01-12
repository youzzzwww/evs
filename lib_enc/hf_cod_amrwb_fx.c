/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/

#define L_FIR           31


void hf_cod_init_fx(
    Word16 *mem_hp400_enc,             /* o: memory of hp 400 Hz filter   */
    Word16 *mem_hf1_enc,               /* o: HF band-pass filter memory   */
    Word16 *mem_syn_hf_enc,            /* o: HF synthesis memory          */
    Word16 *mem_hf2_enc,               /* o: HF band-pass filter memory   */
    Word16 *gain_alpha_fx              /* o: smoothing gain for transitions between active and inactive frames */
)
{
    set16_fx( mem_hp400_enc, 0, 6);
    set16_fx( mem_hf1_enc, 0, 2*L_FILT16k);
    set16_fx( mem_syn_hf_enc, 0, M16k);
    set16_fx( mem_hf2_enc, 0, 2*L_FILT16k);

    *gain_alpha_fx = 16384;
    move16();

    return;
}

void hf_cod_fx(
    const Word32 core_brate_fx,                     /* i  : core bitrate                 */
    const Word16 *speech16k_fx,                     /* i  : original speech at 16 kHz    */
    const Word16 Aq_fx[],                           /* i  : quantized Aq                 */
    const Word16 exc_fx[],                          /* i  : excitation at 12.8 kHz       */
    Word16 synth_fx[],                        /* i  : 12.8kHz synthesis signal     */
    Word16 *seed2_enc_fx,                     /* i/o: random seed for HF noise gen */
    Word16 *mem_hp400_enc_fx,                 /* i/o: memory of hp 400 Hz filter   */
    Word16 *mem_syn_hf_enc_fx,                /* i/o: HF synthesis memory          */
    Word16 *mem_hf1_enc_fx,                   /* i/o: HF band-pass filter memory   */
    Word16 *mem_hf2_enc_fx,                   /* i/o: HF band-pass filter memory   */
    const Word16 dtxHangoverCount_fx,
    Word16 *gain_alpha_fx,                    /* i/o: smoothing gain for transitions between active and inactive frames */
    Word16 *hf_gain_fx,                        /* o  :  HF gain to be transmitted to decoder */
    Word16 Q_exc,
    Word16 Q_syn
)
{
    /*------------------------Scaling-------------------------------------------------
    speech16k - Q0
    Aq - Q12
    exc - Q_exc
    synth - Q_syn
    mem_hp400_enc - Q_syn
    mem_syn_hf_enc - Q0
    mem_hf1_enc - Q0
    mem_hf2_enc - Q0
    gain_alpha_fx - Q14
    ener - all dynamic
    fac, scale - Q12
    Ap - Q12
    HF_SP - Q0
    ---------------------------------------------------------------------------------*/

    Word16 i, q1, q2, sign;
    Word16 shift;
    Word16 ener_hf_fx, ener_exc_fx, ener_input_fx, fac_fx, tmp_fx, ener_fx, scale_fx;
    Word16 Ap_fx[M+1];
    Word16 HF_SP_fx[L_SUBFR16k];
    Word16 HF_est_gain_fx, HF_calc_gain_fx, HF_corr_gain_fx, HF_gain_ind_fx;
    Word32 dist_min_fx, dist_fx;
    Word16 HF_fx[L_SUBFR16k], HF_syn_fx[L_SUBFR16k];                        /* o  : HF excitation                */
    Word32 L_tmp;
    Word16 *pt1;
    const Word16 *pt2;

    /* Original speech signal as reference for high band gain quantisation */
    Copy(speech16k_fx, HF_SP_fx, L_SUBFR16k);

    /*-----------------------------------------------------------------*
    * generate white noise vector
    *-----------------------------------------------------------------*/

    Random_Fill(seed2_enc_fx, L_SUBFR16k, HF_fx, 3); /*Q(-3) */

    /*-----------------------------------------------------------------*
    * calculate energy scaling factor so that white noise would have the
    * same energy as exc12k8
    *-----------------------------------------------------------------*/

    ener_exc_fx = dot_prod_satcontr(exc_fx, exc_fx, Q_exc, Q_exc, &q1, L_SUBFR);
    ener_hf_fx = dot_prod_satcontr(HF_fx, HF_fx, -3, -3, &q2, L_SUBFR16k);

    scale_fx = div_s(shl(1, 14), ener_exc_fx); /*Q(29-q1) */
    L_tmp = L_mult(ener_hf_fx, scale_fx); /*30-q1+q2 */
    q2 = sub(q1, q2); /*30-q2 */
    scale_fx = round_fx(Isqrt(L_shl(L_tmp, sub(q2, 26)))); /*Q13 */


    pt1 = HF_fx;
    FOR( i = 0; i < L_SUBFR16k; i++ )
    {
        *pt1 = round_fx(L_shl(L_mult((*pt1), scale_fx), 5)); /*Q0 */
        pt1++;
    }

    /*-----------------------------------------------------------------*
    * calculate energy scaling factor to respect tilt of synth12k8
    * (tilt: 1=voiced, -1=unvoiced)
    *-----------------------------------------------------------------*/

    hp400_12k8_fx( synth_fx, L_SUBFR, mem_hp400_enc_fx ); /*synth_fx: Q(Q_syn-4) */

    ener_fx = dot_prod_satcontr(&synth_fx[1], &synth_fx[1], sub(Q_syn, 4), sub(Q_syn, 4), &q1, sub(L_SUBFR, 1));
    tmp_fx = dot_prod_satcontr(&synth_fx[1], synth_fx, sub(Q_syn ,4), sub(Q_syn, 4), &q2, sub(L_SUBFR, 1));

    IF ( sub(abs_s(tmp_fx), ener_fx )>=0 )
    {
        tmp_fx = shr(tmp_fx, 1);
        q2 = sub(q2, 1);
    }

    sign = 0;
    move16();
    IF ( tmp_fx < 0 )
    {
        tmp_fx = abs_s(tmp_fx);
        sign = 1;
        move16();
    }

    fac_fx = div_s(tmp_fx, ener_fx); /*Q(15+q2-q1) */
    shift = sub(q1, add(q2, 5));
    fac_fx = shl(fac_fx, shift); /*Q10 */
    IF (sign)
    {
        fac_fx = s_xor(fac_fx, -1);
    }

    HF_est_gain_fx = sub(1024, fac_fx); /*Q12 */

    test();
    IF( L_sub(core_brate_fx, SID_1k75) == 0 || core_brate_fx == FRAME_NO_DATA )
    {
        HF_est_gain_fx = round_fx(L_shl(L_mult(HF_est_gain_fx, 20480), 1)); /*Q10 */
    }

    HF_est_gain_fx = s_max(HF_est_gain_fx, 102);
    HF_est_gain_fx = s_min(HF_est_gain_fx, 1024);

    /*-----------------------------------------------------------------*
    * synthesis of noise: 4.8kHz..5.6kHz --> 6kHz..7kHz
    *-----------------------------------------------------------------*/

    weight_a_lc_fx( Aq_fx, Ap_fx, Gamma_19661_Tbl_fx, M );
    Syn_filt_s( 0, Ap_fx, M, HF_fx, HF_syn_fx, L_SUBFR16k, mem_syn_hf_enc_fx + (M16k - M), 1 ); /*Q0 */

    /*-----------------------------------------------------------------*
    * high pass filtering (0.9375ms of delay = 15 samples@16k)
    *-----------------------------------------------------------------*/

    fir_fx(HF_syn_fx, fir_6k_8k_fx, HF_syn_fx, mem_hf1_enc_fx, L_SUBFR16k, 30, 1, 0);
    fir_fx(HF_SP_fx, fir_6k_8k_fx, HF_SP_fx, mem_hf2_enc_fx, L_SUBFR16k, 30, 1, 0);

    /* check the gain difference */

    ener_hf_fx = dot_prod_satcontr(HF_syn_fx, HF_syn_fx, 0, 0, &q2, L_SUBFR16k);
    ener_input_fx = dot_prod_satcontr(HF_SP_fx, HF_SP_fx, 0, 0, &q1, L_SUBFR16k);

    HF_calc_gain_fx = div_s(shl(1, 14), ener_input_fx); /*Q(29-q1) */
    L_tmp = L_mult(ener_hf_fx, HF_calc_gain_fx); /*30-q1+q2 */
    q2 = sub(q1, q2); /*30-q2 */
    HF_calc_gain_fx = round_fx(Isqrt(L_shl(L_tmp, sub(q2, 20)))); /*Q10  */


    /* set energy of HF synthesis to energy of original HF:
       cross-fade between HF levels in active and inactive frame in hangover period */

    IF ( sub(4, dtxHangoverCount_fx) > 0 )
    {
        *gain_alpha_fx = 16384;
        move16();
    }
    ELSE
    {
        L_tmp = L_shl(L_mult(sub(10, dtxHangoverCount_fx), 4681), 15);/*Q31 */
        L_tmp = Mult_32_16(L_tmp, *gain_alpha_fx); /*Q30 */
        *gain_alpha_fx = round_fx(L_tmp); /*Q14 */
    }
    L_tmp = L_mult(sub(16384, *gain_alpha_fx), HF_est_gain_fx); /*Q25 */
    L_tmp = L_mac(L_tmp, *gain_alpha_fx, HF_calc_gain_fx); /*Q25  */
    IF (L_sub(L_tmp,67108863) >= 0 )
    {
        L_tmp = 67108863;
        move32();
    }
    L_tmp = L_shl(L_tmp, 5);
    HF_corr_gain_fx = extract_h(L_tmp); /*Q14; HF_corr_gain_fx/2 in Q15 */

    /* Quantize the correction gain */
    dist_min_fx = 2147483647;
    move32();
    HF_gain_ind_fx = 0;
    move16();
    pt2 = HP_gain_fx;
    FOR ( i = 0; i < 16; i++ )
    {
        dist_fx = L_mult(sub(HF_corr_gain_fx, *pt2), sub(HF_corr_gain_fx, *pt2));
        pt2++;
        IF ( L_sub(dist_min_fx, dist_fx) > 0 )
        {
            dist_min_fx = L_add(dist_fx, 0);
            HF_gain_ind_fx = i;
            move16();
        }
    }
    *hf_gain_fx = HF_gain_ind_fx;
    move16();

    return;
}
