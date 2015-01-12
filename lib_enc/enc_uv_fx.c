/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * encod_unvoiced()
 *
 * Encode unvoiced (UC) frames
 *-------------------------------------------------------------------*/
/*fixed point implementation of unvoiced_encoder*/
void encod_unvoiced_fx(
    Encoder_State_fx *st_fx,				/* i/o: state structure                         */
    LPD_state   *mem,                /* i/o: acelp memories                          */
    const Word16 *speech_fx,				/* i  : Input speech							*/
    const Word16 Aw_fx[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 *Aq_fx,					/* i  : 12k8 Lp coefficient                     */
    const Word16 vad_flag_fx,
    const Word16 *res_fx,					/* i  : residual signal                         */
    Word16 *syn_fx,                 /* o  : core synthesis                          */
    Word16 *tmp_noise_fx,           /* o  : long-term noise energy                  */
    Word16 *exc_fx,                 /* i/o: current non-enhanced excitation         */
    Word16 *pitch_buf_fx,           /* o  : floating pitch values for each subframe */
    Word16 *voice_factors_fx,       /* o  : voicing factors                         */
    Word16 *bwe_exc_fx,              /* i/o: excitation for SWB TBE                 */
    const Word16 Q_new,
    const Word16 shift
)
{
    Word16 xn_fx[L_SUBFR];             /* Target vector for pitch search     */
    Word16 h1_fx[L_SUBFR];             /* Impulse response vector            */
    Word16 code_fx[L_SUBFR];           /* Fixed codebook excitation          */
    Word16 y2_fx[L_SUBFR];             /* Filtered algebraic excitation      */
    Word16 *pt_pitch_fx;               /* pointer to floating pitch buffer   */
    Word16 gain_pit_fx;                /* Pitch gain                         */
    Word16 voice_fac_fx;               /* Voicing factor                     */
    Word32 L_gain_code_fx;             /* gain of code                       */
    Word16 gain_inov_fx;               /* inovative gain                     */
    const Word16 *p_Aw_fx, *p_Aq_fx;   /* pointer to LP filter coeff. vector */
    Word16 i_subfr;
    Word32 norm_gain_code_fx;

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/
    gain_pit_fx = 0;
    move16();

    test();
    test();
    test();
    IF( st_fx->Opt_SC_VBR_fx && vad_flag_fx == 0 && (sub(st_fx->last_ppp_mode_fx,1) == 0 || sub(st_fx->last_nelp_mode_fx,1) == 0) )
    {
        /* SC_VBR - reset the encoder, to avoid memory not updated issue for the
           case when UNVOICED mode is used to code inactive speech */
        CNG_reset_enc_fx( st_fx, mem, pitch_buf_fx, voice_factors_fx);
    }

    p_Aw_fx = Aw_fx;
    p_Aq_fx = Aq_fx;
    pt_pitch_fx = pitch_buf_fx;
    move16();

    FOR (i_subfr=0; i_subfr<L_FRAME; i_subfr+=L_SUBFR)
    {
        /*----------------------------------------------------------------*
         * Bandwidth expansion of A(z) filter coefficients
         * Find the excitation search target "xn" and innovation target in residual domain "cn"
         * Compute impulse response, h1[], of weighted synthesis filter
         *----------------------------------------------------------------*/
        Copy( &res_fx[i_subfr], &exc_fx[i_subfr], L_SUBFR );

        find_targets_fx( speech_fx, mem->mem_syn, i_subfr, &mem->mem_w0, p_Aq_fx,
                         res_fx, L_SUBFR, p_Aw_fx, st_fx->preemph_fac, xn_fx, NULL
                         ,h1_fx
                       );
        /*Copy_Scale_sig(h1_fx, h2_fx, L_SUBFR, -2);*/
        Scale_sig(h1_fx, L_SUBFR, add(1, shift)); /* set h1[] in Q14 with scaling for convolution */

        /* scaling of xn[] to limit dynamic at 12 bits */
        Scale_sig(xn_fx, L_SUBFR, shift);
        /*----------------------------------------------------------------*
         * Unvoiced subframe processing
         *----------------------------------------------------------------*/

        *pt_pitch_fx = gaus_encode_fx( st_fx, i_subfr, h1_fx, xn_fx, exc_fx, &mem->mem_w0, st_fx->clip_var_fx,
                                       &mem->tilt_code, code_fx, &L_gain_code_fx, y2_fx, &gain_inov_fx,
                                       &voice_fac_fx, &gain_pit_fx, Q_new,shift,&norm_gain_code_fx, st_fx->core_brate_fx );

        *tmp_noise_fx = extract_h(norm_gain_code_fx);

        voice_factors_fx[i_subfr/L_SUBFR] =  0;
        move16();

        interp_code_5over2_fx( &exc_fx[i_subfr], &bwe_exc_fx[i_subfr * HIBND_ACB_L_FAC], L_SUBFR );

        /*-----------------------------------------------------------------*
         * Synthesize speech to update mem_syn[].
         * Update A(z) filters
         *-----------------------------------------------------------------*/
        Syn_filt_s(1, p_Aq_fx, M, &exc_fx[i_subfr], &syn_fx[i_subfr], L_SUBFR, mem->mem_syn, 1 );

        p_Aw_fx += (M+1);
        p_Aq_fx += (M+1);
        pt_pitch_fx++;
    }

    /* SC-VBR */
    st_fx->prev_ppp_gain_pit_fx = gain_pit_fx;
    move16();
    st_fx->prev_tilt_code_fx = mem->tilt_code;
    move16();

    return;
}
