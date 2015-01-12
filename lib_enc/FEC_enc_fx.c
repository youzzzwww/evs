/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"



/*============================================================================*/
/* FUNCTION      : void FEC_encode_fx()										  */
/*----------------------------------------------------------------------------*/
/* PURPOSE       : Encoder supplementary information for FEC				  */
/*----------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :													      */
/* Word16 *synth         i  : pointer to synthesized speech for E computation */
/* Word16 coder_type     i  : type of coder                                   */
/* Word16 clas           i  : signal clas for current frame                   */
/* Word16 *fpit          i  : close loop fractional pitch buffer              */
/* Word16 *res           i  : LP residual signal frame                        */
/* Word16 L_frame        i  : Frame length                                    */
/* Word32  total_brate   i  : total codec bitrate                             */
/*----------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														  */
/*----------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													  */
/* Word16 *last_pulse_pos    i/o: Position of the last pulse                  */
/* Encoder_State_fx *st_fx    i/o: state structure							  */
/*----------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														  */
/*					 _ None													  */
/*----------------------------------------------------------------------------*/
/*																			  */
/*============================================================================*/
void FEC_encode_fx(
    Encoder_State_fx *st_fx,            /* i/o: encoder state structure */
    const Word16 *synth,            /* i  : pointer to synthesized speech for E computation */
    const Word16 coder_type,        /* i  : type of coder                                   */
    Word16 clas,              /* i  : signal clas for current frame                   */
    const Word16 *fpit,             /* i  : close loop fractional pitch buffer              */
    const Word16 *res,              /* i  : LP residual signal frame                        */
    Word16 *last_pulse_pos,   /* i/o: Position of the last pulse                      */
    const Word16 L_frame,           /* i  : Frame length                                    */
    const Word32 total_brate,       /* i  : total codec bitrate                             */
    const Word32  core_brate,       /* i  : total codec bitrate                             */
    const Word16 Q_new,             /* i  : input scaling                                   */
    const Word16 shift              /* i  : scaling to get 12bits                           */
)
{
    Word16 tmpS, index;
    Word16 maxi, sign, tmp_FER_pitch;
    Word32 enr_q, Ltmp;
    Word16 enr_lg_ent, enr_lg_frac, exp_enrq;


    tmpS = 0;
    move16();
    enr_q = 1;
    move16();
    sign = 0;
    move16();
    test();
    test();
    IF( sub(coder_type,UNVOICED) > 0 && sub(coder_type,AUDIO) < 0 && L_sub(core_brate,ACELP_11k60) >= 0 )
    {
        /*-----------------------------------------------------------------*
         * encode signal class (not needed for VC mode since it is clearly voiced) (2 bits)
         *-----------------------------------------------------------------*/
        IF ( sub(coder_type,VOICED) != 0 )
        {
            /* encode signal clas with 2 bits */
            test();
            IF(sub(clas,UNVOICED_CLAS) == 0 )
            {
                index = 0;
                move16();
            }
            ELSE IF( sub(clas,VOICED_TRANSITION) == 0 || sub(clas,UNVOICED_TRANSITION) == 0 )
            {
                index = 1;
                move16();
            }
            ELSE IF( sub(clas,VOICED_CLAS) == 0 )
            {
                index = 2;
                move16();
            }
            ELSE
            {
                index = 3;
                move16();
            }
            push_indice_fx( st_fx, IND_FEC_CLAS, index, FEC_BITS_CLS);
        }

        /*-----------------------------------------------------------------*
         * encode frame energy (5 bits)
         *-----------------------------------------------------------------*/
        test();
        IF( L_sub(total_brate,ACELP_16k40) >= 0 && sub(coder_type,TRANSITION) != 0)    /* GENERIC and VOICED frames */
        {
            /* frame energy (maximum energy per pitch period for voiced frames or mean energy per sample over 2nd halframe for unvoiced frames) */
            /*frame_ener( L_frame, clas, synth, fpit[(L_frame>>6)-1], &enr_q, 0 );*/
            exp_enrq = frame_ener_fx( L_frame, clas, synth, shr_r(fpit[sub(shr(L_frame , 6),1)],6), &enr_q, L_frame, Q_new, shift, 1);

            /* linearly quantize the energy in the range 0 : FEC_ENR_STEP : 96 dB */
            /*tmpS = (short)( 10.0 * log10( enr_q + 0.001f ) / FEC_ENR_STEP )*/ /*To be converted fl_2_fx*/

            enr_lg_frac = Log2_norm_lc(enr_q);
            enr_lg_ent = sub(30, exp_enrq);
            Ltmp = Mpy_32_16(enr_lg_ent,enr_lg_frac, LG10_s3_0);
            tmpS = extract_h(L_shl(Ltmp, 1));

            tmpS = s_min(tmpS, 31);
            tmpS = s_max(tmpS, 0);

            push_indice_fx( st_fx, IND_FEC_ENR, tmpS, FEC_BITS_ENR );
        }
        /*-----------------------------------------------------------------*
        * Encode last glottal pulse position (8 bits)
         *-----------------------------------------------------------------*/
        test();
        IF( L_sub(total_brate,ACELP_32k) >= 0 && sub(coder_type,TRANSITION) != 0)   /* GENERIC frames */
        {
            /* retrieve the last glottal pulse position of the previous frame */
            /* use the current pitch information to scale or not the quantization */
            tmp_FER_pitch = shr(fpit[0],6); /* take the 1st subframe pit, since it is easier to get on decoder side */
            sign = 0;
            move16();
            maxi = *last_pulse_pos;
            move16();
            IF ( maxi < 0 )
            {
                sign = 1;
                move16();
                /*maxi = -maxi; */
                maxi = negate(maxi);
            }

            if ( sub(tmp_FER_pitch,128) >= 0)
            {
                maxi = shr(maxi , 1);
            }

            if ( sub(maxi,127) > 0)
            {
                /* better not use the glottal pulse position at all instead of using a wrong pulse */
                /* can happen only with pitch > 254 and max pit = 289 and should happen very rarely */
                maxi = 0;
                move16();
            }

            if( sign == 1 )
            {
                maxi = add(maxi,128);/* use 8 bits (MSB represent the sign of the pulse) */
            }

            push_indice_fx( st_fx, IND_FEC_POS, maxi, FEC_BITS_POS );
        }
        maxi = 0;
        move16();

        /* If bitrate < 24k4, then the pitch
        is not represented in the same domain (12.k instead of 16k) */
        test();
        IF( sub(clas,VOICED_CLAS) >= 0 && L_sub(total_brate,ACELP_24k40) >= 0 )
        {
            /*maxi = findpulse( L_frame, res, (short)(fpit[(L_frame>>6)-1]), 0, &sign ); */
            maxi = findpulse_fx( L_frame, res, shr_r(fpit[sub(shr(L_frame , 6) , 1)], 6), 0, &sign );
            if ( sign == 1 )
            {
                /*maxi = -maxi;*/
                maxi = negate(maxi);
            }
        }

        *last_pulse_pos = maxi;
        move16();
    }
    ELSE
    {
        *last_pulse_pos = 0;
        move16();
    }

    return;
}


/* TO BE CHECKED !!*/
/*-------------------------------------------------------------------*
* FEC_lsf_estim_enc_fx()
*
* Simulates LSF estimation in case of FEC in the encoder ( only one frame erasure is considered )
* The estimated LSF vector is then used to check LSF stability and may invoke safety-net usage in the next frame
*-------------------------------------------------------------------*/

void FEC_lsf_estim_enc_fx(
    Encoder_State_fx *st_fx,              /* i  : Encoder static memory                        */
    const Word16 L_frame,          /* i  : length of the frame                          */
    Word16 *lsf              /* o  : estimated LSF vector                         */
)
{
    Word16 i;
    Word16 alpha, lsf_mean[M];
    Word16 tmp;

    IF( sub(L_frame, L_FRAME) == 0 )
    {
        Copy( UVWB_Ave_fx, lsf_mean, M );
    }
    ELSE
    {
        Copy( GEWB2_Ave_fx, lsf_mean, M );
    }

    /*----------------------------------------------------------------------*
     * Initialize the alpha factor
     *----------------------------------------------------------------------*/

    IF( sub(st_fx->last_coder_type_fx, UNVOICED) == 0 )
    {
        /* clearly unvoiced */
        alpha = _ALPHA_UU_FX;
        move16();
    }
    ELSE
    {
        test();
        test();
        IF( sub(st_fx->last_coder_type_fx, AUDIO) == 0 || sub(st_fx->clas_fx, INACTIVE_CLAS) == 0 )
        {
            alpha = 32604;
            move16();
        }
        ELSE IF( sub(st_fx->clas_fx, UNVOICED_CLAS) == 0 )
        {
            /* if stable, do not flatten the spectrum in the first erased frame  */
            /* alpha = st->stab_fac * (1.0f - 2.0f*ALPHA_U) + 2.0f*ALPHA_U;     */
            alpha = add(mult(st_fx->stab_fac_fx, 32768 - _ALPHA_U_FX_X_2), _ALPHA_U_FX_X_2);
        }
        ELSE IF( sub(st_fx->clas_fx, UNVOICED_TRANSITION) == 0 )
        {
            alpha = _ALPHA_UT_FX;
            move16();
        }
        ELSE IF( sub(st_fx->clas_fx, VOICED_CLAS) == 0 || sub(st_fx->clas_fx, ONSET) == 0 )
        {
            /* clearly voiced -  mild convergence to the CNG spectrum for the first three erased frames */
            alpha = _ALPHA_V_FX;
            move16();
        }
        ELSE IF( sub(st_fx->clas_fx, SIN_ONSET) == 0 )
        {
            alpha = _ALPHA_S_FX;
            move16();
        }
        ELSE
        {
            /* long erasures and onsets - rapid convergence to the CNG spectrum */
            alpha = _ALPHA_VT_FX;
            move16();
        }
    }
    /*----------------------------------------------------------------------*
     * Extrapolate LSF vector
     *----------------------------------------------------------------------*/
    tmp =  sub(32767, alpha);
    /* extrapolate the old LSF vector */
    FOR (i=0; i<M; i++)
    {
        /* calculate mean LSF vector */
        /*lsf_mean[i] = BETA_FEC * lsf_mean[i] + (1-BETA_FEC) * st->lsf_adaptive_mean[i]; */
        lsf_mean[i] = mac_r(L_mult(BETA_FEC_FX, lsf_mean[i]), 32768-BETA_FEC_FX, st_fx->lsf_adaptive_mean_fx[i]);

        /* move old LSF vector towards the mean LSF vector */
        /* lsf[i] = alpha * st->lsf_old[i] + (1.0f - alpha) * lsf_mean[i]; */
        lsf[i] = mac_r(L_mult(alpha, st_fx->lsf_old_fx[i]), tmp, lsf_mean[i]);
    }

    /* check LSF stability through LSF ordering */
    IF( sub(L_frame, L_FRAME) == 0 )
    {
        reorder_lsf_fx( lsf, MODE1_LSF_GAP_FX, M , INT_FS_FX);
    }
    ELSE  /* L_frame == L_FRAME16k */
    {
        reorder_lsf_fx( lsf, MODE1_LSF_GAP_FX, M, INT_FS_16k_FX);
    }

    return;
}
