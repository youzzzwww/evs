/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

#ifdef _MSC_VER
void E_LPC_isf_isp_conversion(const Word16 isf[], Word16 isp[], const Word16 m);
#endif


/*---------------------------------------------------------------------*
* isf_dec_amr_wb()
*
* Decoding of ISF parameters in AMR-WB IO mode
*---------------------------------------------------------------------*/

void isf_dec_amr_wb_fx(
    Decoder_State_fx *st,          /* i/o: State structure                             */
    Word16 *Aq,          /* o  : quantized A(z) for 4 subframes              */
    Word16 *isf_new,     /* o  : de-quantized ISF vector                     */
    Word16 *isp_new      /* o  : de-quantized ISP vector                     */
)
{
    Word16 i;
    Word16 indice[7];
    Word32 L_tmp;

    set16_fx( indice, -1, 7 );

    /*---------------------------------*
     * ISF de-quantization of SID frames
     *---------------------------------*/

    IF ( L_sub(st->core_brate_fx,SID_1k75) == 0 )
    {

        indice[0] = (Word16)get_next_indice_fx( st, 6 );
        move16();
        indice[1] = (Word16)get_next_indice_fx( st, 6 );
        move16();
        indice[2] = (Word16)get_next_indice_fx( st, 6 );
        move16();
        indice[3] = (Word16)get_next_indice_fx( st, 5 );
        move16();
        indice[4] = (Word16)get_next_indice_fx( st, 5 );
        move16();

        disf_ns_28b_fx( indice, isf_new );

        reorder_isf_fx( isf_new, ISF_GAP_FX, M, Fs_2);

        E_LPC_isf_isp_conversion( isf_new, isp_new, M);
        /* return if SID frame (conversion to A(z) done in the calling function) */
        return;
    }

    /*-----------------------------------------------------------------*
     * ISF de-quantization of all other frames
     *-----------------------------------------------------------------*/

    IF( L_sub(st->core_brate_fx,ACELP_6k60) == 0 )
    {
        indice[0] = (Word16)get_next_indice_fx( st, 8 );
        move16();
        indice[1] = (Word16)get_next_indice_fx( st, 8 );
        move16();
        indice[2] = (Word16)get_next_indice_fx( st, 7 );
        move16();
        indice[3] = (Word16)get_next_indice_fx( st, 7 );
        move16();
        indice[4] = (Word16)get_next_indice_fx( st, 6 );
        move16();

        disf_2s_36b_fx( indice, isf_new, st->mem_AR_fx, st->mem_MA_fx, 1 );
    }
    ELSE
    {
        indice[0] = (Word16)get_next_indice_fx( st, 8 );
        move16();
        indice[1] = (Word16)get_next_indice_fx( st, 8 );
        move16();
        indice[2] = (Word16)get_next_indice_fx( st, 6 );
        move16();
        indice[3] = (Word16)get_next_indice_fx( st, 7 );
        move16();
        indice[4] = (Word16)get_next_indice_fx( st, 7 );
        move16();
        indice[5] = (Word16)get_next_indice_fx( st, 5 );
        move16();
        indice[6] = (Word16)get_next_indice_fx( st, 5 );
        move16();

        disf_2s_46b_fx( indice, isf_new, st->mem_AR_fx, st->mem_MA_fx,1 );
    }
    reorder_isf_fx( isf_new, ISF_GAP_FX, M, Fs_2 );
    /* convert quantized ISFs to ISPs */
    E_LPC_isf_isp_conversion( isf_new, isp_new, M);

    /*-------------------------------------------------------------------------------------*
     * FEC - update adaptive mean ISF vector
     *-------------------------------------------------------------------------------------*/

    FOR ( i=0; i<M; i++ )
    {
        /*st->lsf_adaptive_mean[i] = (st->lsfoldbfi1[i] + st->lsfoldbfi0[i] + isf_new[i]) / 3;*/
        L_tmp = L_mult(st->lsfoldbfi1_fx[i], 10923);
        L_tmp = L_mac(L_tmp, st->lsfoldbfi0_fx[i], 10923);
        st->lsf_adaptive_mean_fx[i] = round_fx(L_mac(L_tmp, isf_new[i], 10923));
    }

    /*-------------------------------------------------------------------------------------*
     * ISP interpolation
     * A(z) calculation
     *-------------------------------------------------------------------------------------*/
    if(st->rate_switching_reset)
    {
        /*extrapolation instead of interpolation*/
        Copy(isp_new, st->lsp_old_fx, M);
        Copy(isf_new, st->lsf_old_fx, M);
    }

    /* ISP interpolation and A(z) calculation */
    int_lsp_fx( L_FRAME, st->lsp_old_fx, isp_new, Aq, M, 0, interpol_isp_amr_wb_fx, 1 );

    /*------------------------------------------------------------------*
     * Check ISF stability : distance between old ISF and current ISF
     *------------------------------------------------------------------*/

    st->stab_fac_fx = lsf_stab_fx( isf_new, st->lsf_old_fx, 1, st->L_frame_fx );

    return;
}

/*-------------------------------------------------------------------*
 * disf_ns_28b()
 *
 * ISF de-quantizer for SID_1k75 frames (only for AMR-WB IO mode)
 *-------------------------------------------------------------------*/

void disf_ns_28b_fx(
    Word16 *indice,      /* i  : quantized indices (use indice[0] = -1 in the decoder) */
    Word16 *isf_q        /* o  : ISF in the frequency domain (0..6400) */
)
{
    Word16 i;

    FOR (i = 0; i < 2; i++)
    {
        isf_q[i] =  dico1_ns_28b_fx[indice[0]*2+i];
        move16();
    }

    FOR (i = 0; i < 3; i++)
    {
        isf_q[i+2] =  dico2_ns_28b_fx[indice[1]*3+i];
        move16();
        isf_q[i+5] =  dico3_ns_28b_fx[indice[2]*3+i];
        move16();
    }

    FOR (i = 0; i < 4; i++)
    {
        isf_q[i+8] =  dico4_ns_28b_fx[indice[3]*4+i];
        move16();
        isf_q[i+12] =  dico5_ns_28b_fx[indice[4]*4+i];
        move16();
    }

    FOR (i=0; i<M; i++)
    {
        isf_q[i] = add(isf_q[i] , mean_isf_noise_amr_wb_fx[i]);
        move16();
    }

    return;
}

/*-------------------------------------------------------------------*
 * disf_2s_46b()
 *
 * ISF de-quantizer for 46b. codebooks (only for AMR-WB IO mode)
 *-------------------------------------------------------------------*/

void disf_2s_46b_fx(
    Word16 *indice,    /* i  : quantized indices (use indice[0] = -1 in the decoder) */
    Word16 *isf_q,     /* o  : quantized ISFs in the cosine domain */
    Word16 *mem_AR,    /* o  : quantizer memory for AR model       */
    Word16 *mem_MA,    /* i/o: quantizer memory for MA model       */
    const Word16 enc_dec     /* i  : encoder (0), decoder (1) G722.2 FER          */
)
{
    Word16 i;

    IF (enc_dec != 0) /* Redirection for G722.2 compatibility */
    {
        i = 0;
        move16();
        WHILE (sub(Indirect_dico1[i], indice[0]) != 0)
        {
            i = add(i, 1);
        }
        indice[0] = i;
        move16();
    }

    FOR (i = 0; i < 9; i++)
    {
        isf_q[i] =  dico1_isf_fx[indice[0]*9+i];
        move16();
    }

    FOR (i = 0; i < 7; i++)
    {
        isf_q[i+9] =  dico2_isf_fx[indice[1]*7+i];
        move16();
    }

    FOR (i = 0; i < 3; i++)
    {
        isf_q[i] =  add(isf_q[i],dico21_isf_46b_fx[indice[2]*3+i]);
        move16();
        isf_q[i+3] =  add(isf_q[i+3], dico22_isf_46b_fx[indice[3]*3+i]);
        move16();
        isf_q[i+6] =  add(isf_q[i+6], dico23_isf_46b_fx[indice[4]*3+i]);
        move16();
        isf_q[i+9] =  add(isf_q[i+9], dico24_isf_46b_fx[indice[5]*3+i]);
        move16();
    }

    FOR (i = 0; i < 4; i++)
    {
        isf_q[i+12] =  add(isf_q[i+12], dico25_isf_46b_fx[indice[6]*4+i]);
        move16();
    }

    FOR (i = 0; i < M; i++)
    {
        mem_AR[i] = add(isf_q[i], mult_r(MU_MA_FX, mem_MA[i]));
        move16();     /* Update with quantized ISF vector for AR model */
        mem_MA[i] = isf_q[i];
        move16();                                 /* Update with quantized prediction error for MA model */
        isf_q[i] = add(mem_AR[i], mean_isf_amr_wb_fx[i]);
        move16();         /* Quantized ISFs */
    }

    return;
}


/*-------------------------------------------------------------------*
 * disf_2s_36b()
 *
 * ISF de-quantizer for 36b. codebooks (only for AMR-WB IO mode)
 *-------------------------------------------------------------------*/

void disf_2s_36b_fx(
    Word16 *indice,    /* i  : quantized indices (use indice[0] = -1 in the decoder) */
    Word16 *isf_q,     /* o  : quantized ISFs in the cosine domain */
    Word16 *mem_AR,    /* i/o: quantizer memory for AR model       */
    Word16 *mem_MA,    /* i/o: quantizer memory for MA model       */
    const Word16 enc_dec     /* i  : encoder (0), decoder (1) G722.2 FER          */
)
{
    Word16 i;
    const Word16 *pt_dico1;

    IF (enc_dec != 0) /* Redirection for G722.2 interoperability  */
    {
        i = 0;
        move16();
        WHILE (sub(Indirect_dico1[i], indice[0]) != 0)
        {
            i = add(i,1);
        }
        indice[0] = i;
        move16();
    }


    pt_dico1 = dico1_isf_fx;    /* Pointer of the 1st stage, 1st plit */

    FOR (i = 0; i < 9; i++)
    {
        isf_q[i] =  pt_dico1[indice[0]*9+i];
        move16();
    }

    FOR (i = 0; i < 7; i++)
    {
        isf_q[i+9] =  dico2_isf_fx[indice[1]*7+i];
        move16();
    }

    FOR (i = 0; i < 5; i++)
    {
        isf_q[i] =  add(isf_q[i], dico21_isf_36b_fx[indice[2]*5+i]);
        move16();
    }

    FOR (i = 0; i < 4; i++)
    {
        isf_q[i+5] =  add(isf_q[i+5], dico22_isf_36b_fx[indice[3]*4+i]);
        move16();
    }

    FOR (i = 0; i < 7; i++)
    {
        isf_q[i+9] =  add(isf_q[i+9], dico23_isf_36b_fx[indice[4]*7+i]);
        move16();
    }

    FOR (i = 0; i < M; i++)
    {
        mem_AR[i] = add(isf_q[i], mult_r(MU_MA_FX, mem_MA[i]));
        move16(); /* Update with quantized ISF vector for AR model */
        mem_MA[i] = isf_q[i];
        move16();   /* Update with quantized prediction error for MA model */
        isf_q[i] = mem_AR[i] + mean_isf_amr_wb_fx[i];
        move16();     /* Quantized ISFs */
    }

    return;
}
