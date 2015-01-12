/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h" /* Compilation switches */
#include "cnst_fx.h" /* Common constants */
#include "prot_fx.h" /* Function prototypes */
#include "stl.h"

/*==============================================================================*/
/* FUNCTION : encod_nelp_fx()										        	*/
/*------------------------------------------------------------------------------*/
/* PURPOSE :   Encode Unvoiced frames in SC-VBR                                 */
/*------------------------------------------------------------------------------*/
/*  INPUT ARGUMENTS :												    		*/
/* _ (Encoder_State_fx) st_fx: state structure                                  */
/* _ (Word16[]) speech_fx	: input speech                      Q_new-1         */
/* _ (Word16[]) Aq_fx		: 12k8 Lp coefficient		    	Q12				*/
/* _ (Word16[]) A_fx		: unquantized A(z) filter               			*/
/*                            with bandwidth expansion 		    Q12	    		*/
/* _ (Word16) coder_type_fx	: coding type										*/
/* _ (Word16[]) res_fx		: residual signal                   Q_new           */
/* _ (Word16*) Q_new		: res qformat										*/
/* _ (Word16)  shift                                                            */
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :															*/
/* _ (Word16[]) synth_fx	: core synthesis				                    */
/* _ (Word16[]) tmp_noise_fx: long-term noise energy            Q0              */
/* _ (Word16[]) exc_fx		: current non-enhanced excitation   Q_new			*/
/* _ (Word16[]) exc2_fx		: current enhanced excitation       Q_new			*/
/* _ (Word16[]) pitch_buf_fx: floating pitch values for each subframe Q6		*/
/* _ (Word16*)  voice_factors : voicing factors                                 */
/* _ (Word16*)  bwe_exc       : excitation for SWB TBE                          */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :															*/
/* _ None																		*/
/*==============================================================================*/
void encod_nelp_fx(
    Encoder_State_fx *st_fx,                /* i/o: state structure */
    LPD_state   *mem,                /* i/o: acelp memories             */
    const Word16 *speech_fx,             /* i  : input speech */
    const Word16 Aw_fx[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq_fx[],                 /* i  : 12k8 Lp coefficient                               */
    Word16 *res_fx,                      /* o  : residual signal */
    Word16 *synth_fx,                    /* o  : core synthesis */
    Word16 *tmp_noise_fx,                /* o  : long-term noise energy */
    Word16 *exc_fx,                      /* i/o: current non-enhanced excitation */
    Word16 *exc2_fx,                     /* i/o: current enhanced excitation */
    Word16 *pitch_buf_fx,                 /* o  : floating pitch values for each subframe */
    Word16* voice_factors_fx,             /* o  : voicing factors */
    Word16* bwe_exc_fx,                    /* o  : excitation for SWB TBE */
    Word16 Q_new,
    Word16 shift

)
{
    Word16 xn_fx[L_SUBFR];       /* Target vector for pitch search */
    Word16 h1_fx[L_SUBFR];       /* Impulse response vector */
    Word16 exc_nelp_fx[L_FRAME];

    Word16 i_subfr, j;

    const Word16 *p_Aw_fx, *p_Aq_fx; /* pointer to LP filter coeff. vector */
    Word16 saved_Q_new = Q_new;

    Word16 reduce_gains = 0;

    IF ( sub(st_fx->bwidth_fx, NB) == 0 && L_sub(st_fx->input_Fs_fx, 16000) >= 0)
    {
        IF (st_fx->last_nelp_mode_fx == 0)
        {
            set16_fx( st_fx->nelp_lp_fit_mem, 0, NELP_LP_ORDER*2 );
        }
        Scale_sig(st_fx->nelp_lp_fit_mem, NELP_LP_ORDER*2, sub(Q_new, st_fx->prev_Q_new));

        pz_filter_sp_fx( num_nelp_lp_fx, den_nelp_lp_fx, res_fx, res_fx, st_fx->nelp_lp_fit_mem, NELP_LP_ORDER, NELP_LP_ORDER, L_FRAME, 3);  /*16-Q of filter coeff*/

    }

    p_Aw_fx = Aw_fx;
    p_Aq_fx = Aq_fx;


    FOR (i_subfr=0; i_subfr<L_FRAME; i_subfr+=L_SUBFR)
    {
        /*----------------------------------------------------------------*
         * - Bandwidth expansion of A(z) filter coefficients
         * - Find the excitation search target "xn" and innovation
         * target in residual domain "cn"
         * - Compute impulse response, h1[], of weighted synthesis filter
         *----------------------------------------------------------------*/
        Copy( &res_fx[i_subfr], &exc_fx[i_subfr], L_SUBFR );

        find_targets_fx( speech_fx, mem->mem_syn, i_subfr, &mem->mem_w0, p_Aq_fx,
                         res_fx, L_SUBFR, p_Aw_fx, TILT_FAC_FX, xn_fx, NULL
                         ,h1_fx
                       );
        /* scale xn[] and h1[] to avoid overflow in dot_product12() */
        Scale_sig(xn_fx, L_SUBFR, shift); /* scaling of xn[] to limit dynamic at 12 bits */

        IF (i_subfr == 0)
        {
            test();
            test();
            IF ( sub(st_fx->Local_VAD, 1 ) == 0 && sub( st_fx->Local_VAD, 25 << 8) < 0  && sub( st_fx->bwidth_fx, NB) == 0 )
            {
                reduce_gains = 1;
            }

            nelp_encoder_fx( st_fx, res_fx, exc_nelp_fx, &Q_new
                             ,reduce_gains
                           );

            Scale_sig(exc_nelp_fx, L_FRAME, (saved_Q_new - Q_new));
        }


        *tmp_noise_fx = 0;
        move16();

        /*-----------------------------------------------------------------*
         * Synthesize speech to update mem_syn[].
         * Update A(z) filters
         *-----------------------------------------------------------------*/
        Syn_filt_s(1, p_Aq_fx, M, &exc_nelp_fx[i_subfr], &synth_fx[i_subfr], L_SUBFR, mem->mem_syn, 1);

        p_Aw_fx += (M+1);
        move16();
        p_Aq_fx += (M+1);
        move16();
        *pitch_buf_fx = L_SUBFR_Q6;
        move16();

        pitch_buf_fx++;
        move16();
    }

    Copy(exc_nelp_fx, exc_fx, L_FRAME);

    /*-----------------------------------------------------------------*
     * Updates: last value of new target is stored in mem_w0
     *-----------------------------------------------------------------*/

    mem->mem_w0 = sub(shr(xn_fx[L_SUBFR-1],shift), shr(exc_fx[L_FRAME-1],1));
    move16();/*Q_new-1 */
    mem->tilt_code = 0;
    move16();/* purely unvoiced */
    st_fx->prev_tilt_code_fx = mem->tilt_code;
    move16();

    Copy(exc_fx, exc2_fx, L_FRAME);

    st_fx->prev_ppp_gain_pit_fx = 0;
    move16();

    st_fx->dm_fx.prev_state = 0;
    move16();
    st_fx->dm_fx.prev_gain_pit[0] = st_fx->prev_ppp_gain_pit_fx;
    move16();

    FOR(j=1; j<5; j++)
    {
        st_fx->dm_fx.prev_gain_pit[j] = st_fx->dm_fx.prev_gain_pit[j-1];
        move16();
    }

    interp_code_5over2_fx( exc2_fx, bwe_exc_fx, L_FRAME );
    set16_fx( voice_factors_fx, 0, NB_SUBFR16k );

    return;
}
