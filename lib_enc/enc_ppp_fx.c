/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches */
#include "cnst_fx.h"       /* Common constants */
#include "prot_fx.h"       /* Function prototypes */
#include "stl.h"
/*Temporary location to be move in prot* when merge is done */
void E_LPC_f_lsp_a_conversion(const Word16 *lsp, Word16 *a, const Word16 m);

/*=======================================================================================*/
/* FUNCTION      :  encod_ppp_fx()                                                       */
/*---------------------------------------------------------------------------------------*/
/* PURPOSE       :                                                                       */
/*---------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                                    */
/*   _ (Word16) speech_fx[],     i : input speech Q_new									 */
/*   _ (Word16) Aq_fx[],         i : 12k8 Lp coefficient Q12							 */
/*   _ (Word16) A_fx[],          i : unquantized A(z) filter with bandwidth expansion Q12*/
/*   _ (Word16) coder_type_fx,   i : coding type										 */
/*   _ (Word16) T_op_fx[],       i : open loop pitch									 */
/*   _ (Word16) voicing_fx[],    i : voicing Q15										 */
/*   _ (Word16) *res_fx,         i : residual signal Q_new								 */
/*   _ (Word16)  Q_new           i : Q factor for res								     */
/*   _ (Word16)  vadsnr_fx       i : SNR for current frame Q7							 */
/*---------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                					 */
/*   _ (Word16) *exc2_fx,        o : current enhanced excitation Q0						 */
/*   _ (Word16) *pitch_buf_fx,   o : floating pitch values for each subframe Q6			 */
/*   _ (Word16) *synth_fx,       o : core synthesis Q-1									 */
/*   _ Encoder_State_fx *st_fx:										 					 */
/*                _ lastLgainE_fx - Q11                              					 */
/*                _ lastHgainE_fx - Q11                              					 */
/*                _ lasterbE_fx   - Q13                              					 */
/*---------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :																 */
/*   _ Encoder_State_fx *st_fx:										 					 */
/*				  _	st_fx->dtfs_enc_xxxx												 */
/*				  _	a nd b in   st_fx->dtfs_enc_Q										 */
/*				  	rest all in                 Q0										 */
/*				  - bump_up_fx    - Q0													 */
/*   _ (Word16) *exc_fx,        o : current enhanced excitation Q0						 */
/*---------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                                    */
/*  _ None.																				 */
/*---------------------------------------------------------------------------------------*/
/* CALLED FROM : TX                                                                      */
/*=======================================================================================*/
void encod_ppp_fx(
    Encoder_State_fx* st_fx,		/* i/o: state structure */
    LPD_state   *mem,                /* i/o: acelp memories             */
    const Word16 speech_fx[],     /* i : input speech Q_new*/
    const Word16 Aw_fx[],                 /* i  : weighted A(z) unquantized for subframes */
    const Word16 Aq_fx[],                 /* i  : 12k8 Lp coefficient                               */
    Word16 *coder_type_fx,		/* i : coding type */
    Word16 sharpFlag_fx,    /* i : formant sharpening flag */
    const Word16 T_op_fx[],       /* i : open loop pitch */
    const Word16 voicing_fx[],    /* i : voicing Q15*/
    Word16 *res_fx,         /* i : residual signal Q_new*/
    Word16 *synth_fx,       /* o : core synthesis Q-1*/
    Word16 *exc_fx,         /* i/o: current non-enhanced excitation Q_new*/
    Word16 *exc2_fx,        /*   o: current enhanced excitation Q0*/
    Word16 *pitch_buf_fx,   /*   o: floating pitch values for each subframe Q6*/
    Word16 *voice_factors,  /* o  : voicing factors */
    Word16 *bwe_exc,        /* o  : excitation for SWB TBE */
    Word16 Q_new,
    Word16 shift
)
{
    Word16 xn_fx[L_SUBFR];        	/* Target vector for pitch search */
    Word16 h1_fx[L_SUBFR+(M+1)];  	/* Impulse response vector */
    Word16 i_subfr;					/* tmp variables */
    const Word16 *p_Aw_fx, *p_Aq_fx;  /* pointer to LP filter coeff. vector*/

    Word16 k;
    Word16 p_Aq_old_fx[M+1], excQ_ppp_fx[L_FRAME] ,p_Aq_curr_fx[M], pitch_fx[NB_SUBFR];
    Word16 LPC_de_old_fx[M+1], LPC_de_curr_fx[M+1];
    Word16 shift_wsp = add(Q_new,shift);
    Word16 rate_ctrl_fx;
    Word16 saved_Q_new = Q_new;

    rate_ctrl_fx = st_fx->rate_control_fx;
    move16();

    /*------------------------------------------------------------------*
    * ACELP subframe loop
    *------------------------------------------------------------------*/
    p_Aw_fx = Aw_fx;
    p_Aq_fx = Aq_fx;
    FOR (i_subfr=0; i_subfr<L_FRAME; i_subfr+=L_SUBFR)
    {
        /*----------------------------------------------------------------*
         * Bandwidth expansion of A(z) filter coefficients
         * Find the the excitation search target "xn" and innovation
         * target in residual domain "cn"
         * Compute impulse response, h1[], of weighted synthesis filter
         *----------------------------------------------------------------*/

        Copy( &res_fx[i_subfr], &exc_fx[i_subfr], L_SUBFR );

        find_targets_fx( speech_fx, mem->mem_syn, i_subfr, &mem->mem_w0, p_Aq_fx,
                         res_fx, L_SUBFR, p_Aw_fx, TILT_FAC_FX, xn_fx, NULL
                         ,h1_fx
                       );

        /* scale xn[] and h1[] to avoid overflow in dot_product12() */
        Scale_sig(xn_fx, L_SUBFR, shift); /* scaling of xn[] to limit dynamic at 12 bits */

        /* call voiced encoder at this point */
        IF (i_subfr == 0) /* generate the L_FRAME exc */
        {
            FOR(k=0; k<M; k++)
            {
                p_Aq_curr_fx[k] = p_Aq_fx[k+(3*(M+1))+1];
                move16();
            }

            E_LPC_f_lsp_a_conversion( st_fx->lsp_old_fx, p_Aq_old_fx, M );
            deemph_lpc_fx( p_Aq_curr_fx, p_Aq_old_fx, LPC_de_curr_fx, LPC_de_old_fx ,1);
            /* both outputs LPC_de_curr_fx and LPC_de_old_fx are in Q12 */


            /* last frame-end lpc and curr frame-end lpc */
            ppp_voiced_encoder_fx( st_fx, res_fx, excQ_ppp_fx, T_op_fx[1], LPC_de_old_fx,
                                   LPC_de_curr_fx, exc_fx, pitch_fx, st_fx->vadsnr_fx, Q_new );

            Scale_sig(exc_fx, L_FRAME, (saved_Q_new - Q_new));
            if (sub(st_fx->bump_up_fx,1) == 0)
            {
                i_subfr = L_FRAME;
                move16();
            }
        }

        IF( sub(st_fx->bump_up_fx,1) != 0 )
        {
            /*-----------------------------------------------------------------*
             * Gain clipping test to avoid unstable synthesis on frame erasure
             * or in case of floating point encoder & fixed p. decoder
             *-----------------------------------------------------------------*/
            gp_clip_fx(voicing_fx, i_subfr, *coder_type_fx, xn_fx, st_fx->clip_var_fx, sub(shift_wsp,1));


            /* run the above to maintain gain clipping memories */
            gp_clip_test_gain_pit_fx( st_fx->prev_ppp_gain_pit_fx, st_fx->clip_var_fx );


            /*-----------------------------------------------------------------*
             * Synthesize speech to update mem_syn[].
             * Update A(z) filters
             *-----------------------------------------------------------------*/

            Syn_filt_s( 1, p_Aq_fx, M, &excQ_ppp_fx[i_subfr], &synth_fx[i_subfr], L_SUBFR, mem->mem_syn, 1 );


            p_Aw_fx += (M+1);
            p_Aq_fx += (M+1);
        }

    }   /* end of subframe loop */

    IF( st_fx->bump_up_fx )
    {
        /* PPP failed, bump up */
        st_fx->ppp_mode_fx = 0;
        move16();
        st_fx->core_brate_fx = ACELP_7k20;
        move16();
        st_fx->pppcountE_fx = 0;
        move16();

        IF ( st_fx->set_ppp_generic_fx )
        {
            *coder_type_fx = GENERIC;
            move16();
        }
        ELSE
        {
            *coder_type_fx = VOICED;
            move16();
        }

        /* We write signalling indices again only in case of bump_up */
        /* delete previous indices */
        reset_indices_enc_fx( st_fx );

        /* signalling matrix (writing of signalling bits) */
        signalling_enc_fx( st_fx, *coder_type_fx, sharpFlag_fx );
    }
    ELSE
    {
        Copy(excQ_ppp_fx, exc_fx, L_FRAME);

        /*-----------------------------------------------------------------*
         * Updates: last value of new target is stored in mem_w0
         *-----------------------------------------------------------------*/

        mem->mem_w0 = sub(shr(xn_fx[L_SUBFR-1],shift), shr(exc_fx[L_FRAME-1],1)); /*Q_new-1 */

        Copy(exc_fx, exc2_fx, L_FRAME);

        st_fx->dm_fx.prev_state = 2;
        move16();/*Q0 dispMem index 0 */
        st_fx->dm_fx.prev_gain_pit[0] = st_fx->prev_ppp_gain_pit_fx;
        move16();/*Q14 dispMem index 2 */

        FOR( k=1; k<5; k++ )
        {
            st_fx->dm_fx.prev_gain_pit[k] = st_fx->dm_fx.prev_gain_pit[k-1];
            move16();
        }

        mem->tilt_code = st_fx->prev_tilt_code_fx;
        move16();
        Copy(pitch_fx, pitch_buf_fx, NB_SUBFR);
        pitch_buf_fx[ NB_SUBFR16k -1 ] = pitch_fx[ NB_SUBFR - 1 ];

        interp_code_5over2_fx( exc2_fx, bwe_exc, L_FRAME );
        set16_fx( voice_factors, 0, NB_SUBFR16k );
    }

    st_fx->rate_control_fx = rate_ctrl_fx;
    move16();

    return;
}
