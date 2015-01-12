/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "cnst_fx.h"
#include "rom_enc_fx.h"
#include "rom_com_fx.h"
#include "stl.h"
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_basop_util.h"



void lpc_quantization(
    Encoder_State_fx * st,
    const Word16 core,
    const Word16 lpcQuantization,
    const Word16 lsfold_q[],
    const Word16 lsp[],
    const Word16 lspmid[],
    Word16 lsp_q[],
    Word16 lsf_q[], /* 14Q1*1.28 */
    Word16 lspmid_q[],
    Word16 lspq_ind[],
    Word16 clip_var[],
    Word16 mem_MA[],
    Word16 mem_AR[],
    const Word8  narrowBand,
    const Word16 coder_type,
    const Word8  acelp_midLpc,
    Word16 param_lpc[],
    Word16 nbits_lpc[],
    Word16 * bits_param_lpc,
    Word16 *no_param_lpc,
    Word16 *seed_acelp,
    Word32 * Bin_Ener,
    Word32 * Bin_Ener_old,
    const Word16 Q_ener
)
{

    Word16 nb_indices;
    Word16 lsfmid_q[M]; /* 14Q1*1.28 */
    Word16 lsfmid_idx;
    Word16 i, force_sf;
    Word16 lsf[M], lsfmid[M];

    Word16 fec_lsf[M], stab;

    nb_indices = 0;
    move16();

    /****** High-rate LPC quantizer *******/

    IF (lpcQuantization==0)
    {
        E_LPC_lsp_lsf_conversion(lsp, lsf, M);

        /* check resonance for pitch clipping algorithm */
        gp_clip_test_lsf_fx( lsf, clip_var, M );

        IF ( (sub(core, TCX_10_CORE) == 0) )
        {
            E_LPC_lsp_lsf_conversion(lspmid, lsfmid, M);
        }

        /* LPC quantizer */
        qlpc_avq(lsf, lsfmid, lsf_q, lsfmid_q, param_lpc, &nb_indices, nbits_lpc, core, st->sr_core);

        E_LPC_lsf_lsp_conversion(lsf_q, lsp_q, M);

        IF( sub(core, TCX_10_CORE) == 0 )
        {
            E_LPC_lsf_lsp_conversion( lsfmid_q, lspmid_q, M );
        }

        /* assert(nb_indices<=NPRM_LPC_NEW); */
    }

    /****** Low-rate LPC quantizer *******/

    ELSE IF ( sub(lpcQuantization, 1) == 0)
    {
        lsp2lsf_fx(lsp, lsf, M, extract_l(st->sr_core));

        gp_clip_test_lsf_fx( lsf, clip_var, M );

        force_sf = 0;
        move16();
        /*Force safety net when possible in case of transitions*/
        test();
        test();
        IF( st->tc_cnt_fx >= 1 || L_sub(st->last_core_brate_fx,SID_2k40) <= 0 || (sub(st->next_force_safety_net_fx ,1) == 0) )
        {
            force_sf = 1;
            move16();
            st->next_force_safety_net_fx = 0;
            move16();
        }

        test();
        IF ( sub(st->next_force_safety_net_fx,1) == 0 && sub(st->Opt_RF_ON,1)==0 )
        {
            force_sf = 1;
            st->next_force_safety_net_fx = 0;
        }

        test();
        IF ( L_sub(st->sr_core, INT_FS_16k)== 0 && sub(coder_type,UNVOICED) == 0 )
        {
            lsf_end_enc_fx( st, lsf, lsf_q, mem_AR, mem_MA, ENDLSF_NBITS, GENERIC, st->bwidth_fx, Bin_Ener, Q_ener, st->sr_core, st->core_brate_fx,
                            &st->streaklimit_fx, &st->pstreaklen_fx, force_sf, 0,
                            1, param_lpc, no_param_lpc, bits_param_lpc, GENERIC );

            nb_indices = *no_param_lpc;
        }
        ELSE
        {
            lsf_end_enc_fx( st, lsf, lsf_q, mem_AR, mem_MA, ENDLSF_NBITS, coder_type, st->bwidth_fx, Bin_Ener, Q_ener, st->sr_core, st->core_brate_fx,
            &st->streaklimit_fx, &st->pstreaklen_fx, force_sf, 0,
            1, param_lpc, no_param_lpc, bits_param_lpc, coder_type );

            nb_indices = *no_param_lpc;
        }


        FEC_lsf_estim_enc_fx( st, st->L_frame_fx, fec_lsf );

        /* FEC - calculate LSF stability */
        stab = lsf_stab_fx( lsf_q, fec_lsf, 0, st->L_frame_fx); /*Q15*/


        test();
        test();
        test();
        IF ( sub(stab,add(STAB_FAC_LIMIT_FX, 6553/* =0.2 in Q15*/)) < 0 &&
             ( sub(coder_type,VOICED) == 0  || sub(coder_type,GENERIC) == 0) && sub(st->Opt_RF_ON,1)==0 )
        {
            st->next_force_safety_net_fx = 1;
        }

        lsf2lsp_fx(lsf_q, lsp_q, M, st->sr_core);

        *nbits_lpc = ENDLSF_NBITS;
        move16();

    }
    ELSE
    {
        assert(0);
    }

    IF (lspq_ind != NULL)
    {
        E_LPC_lsf_lsp_conversion( lsf_q, lspq_ind, M );
    }

    *seed_acelp=0;
    move16();
    FOR(i=nb_indices-1; i>=0; i--)
    {
        *seed_acelp = extract_l(L_mac0(L_mac0(13849, shr(*seed_acelp, 1), 31821), param_lpc[i], 31821));
        move16();
    }

    /* Mid-frame LPC quantization */

    test();
    IF(lpcQuantization && acelp_midLpc)
    {

        IF(st->rate_switching_reset==0)
        {
            lsp2lsf_fx(lspmid, lsfmid, M, extract_l(st->sr_core));

            midlsf_enc ( lsfold_q, lsf_q, lsfmid, &lsfmid_idx, M, Bin_Ener_old, Q_ener, narrowBand, st->sr_core, coder_type );
            param_lpc[nb_indices++] = lsfmid_idx;
            move16();
            midlsf_dec (lsfold_q, lsf_q, lsfmid_idx, lsfmid_q, coder_type
                        ,NULL,0,1
                       );

            reorder_lsf_fx( lsfmid_q, LSF_GAP_MID_FX, M, st->sr_core );
            lsf2lsp_fx(lsfmid_q, lspmid_q, M, st->sr_core);
        }
        ELSE
        {
            param_lpc[nb_indices++] = 0;
            move16();
        }
    }



    return;
}



#define MIN_LOG_FX          0
#define MIN_LOG_VAL_FX   -15360    /* -60.0f in Q8 */

void Unified_weighting_fx(
    Word32 Bin_Ener_128_fx[], /* i  : FFT Bin energy 128 bins in two sets    Q_ener */
    Word16 Q_ener,
    const Word16 lsf_fx[],          /* i  : LSF vector                             x2.56 */
    Word16 w_fx[],            /* o  : LP weighting filter (numerator)         Q8 */
    const Word16 narrowBand,     /* i  : flag for Narrowband                     */
    const Word16 unvoiced,       /* i  : flag for Unvoiced frame                 */
    const Word32 sr_core,        /* i  : sampling rate of core-coder             */
    const Word16   order           /* i  : LP order                                */
)
{
    Word16 i;
    const Word16 (*ptr_lsf_fit_model)[M];
    Word16 last_bin;
    /*float compen;*/

    Word16 exp, frac;
    Word16 w_fft_fx[M]/*, w_fx[M]*/;
    Word16 norm_lsf_fx[M]; /* Q0 */
    Word16 tmp_fx, min_fx, tmp1_fx, tmp2_fx, s1, s2;
    Word32 L_tmp;
    Word16 nf_fx;
    Word32 Bin_Ener_160_fx[160];
    Word32 *Bin_Ener_fx;
    const Word32 *Freq_w_Table_fx;


    /*Config. weighting*/
    IF ( narrowBand )
    {
        ptr_lsf_fit_model = lsf_unified_fit_model_nb;
        nf_fx = 16384;
        move16(); /* x2.56 */

        last_bin = 127;
        move16();
        Bin_Ener_fx = Bin_Ener_128_fx;
    }
    ELSE IF( L_sub(sr_core, 12800) == 0 )
    {
        ptr_lsf_fit_model = lsf_unified_fit_model_wb;
        nf_fx = 16384;
        move16(); /* x2.56 */

        last_bin = 127;
        move16();
        Bin_Ener_fx = Bin_Ener_128_fx;
    }
    ELSE
    {
        ptr_lsf_fit_model = lsf_unified_fit_model_wbhb;
        nf_fx = 20480;
        move16(); /* x2.56 */

        /* Fill the missing part (128~159) of the bin energy */
        last_bin = 159;
        move16();

        Copy32(Bin_Ener_128_fx, Bin_Ener_160_fx, L_FFT/2);

        /* Find average bin energy (32 Energy) */
        L_tmp = L_deposit_l(0);
        FOR ( i=95; i<127; i++ )
        {
            L_tmp = L_add(L_tmp, Bin_Ener_160_fx[i]);
        }

        L_tmp = L_shr(L_tmp, 5);
        FOR ( i=127; i<160; i++ )
        {
            Bin_Ener_160_fx[i] = L_tmp;
            move32();
        }

        Bin_Ener_fx = Bin_Ener_160_fx;
    }

    /* 1) FFT weights*/
    Freq_w_Table_fx = Freq_Weight_Com_fx;
    if ( unvoiced )
    {
        Freq_w_Table_fx = Freq_Weight_UV_fx;
    }

    /* Use Envelope */
    min_fx = MAX_16;
    move16();
    FOR ( i=0; i<M; i++ )
    {
        norm_lsf_fx[i] = mult_r(lsf_fx[i], 256);
        move16();

        IF (norm_lsf_fx[i] == 0)
        {
            L_tmp = L_add(Bin_Ener_fx[1], 0);
        }
        ELSE IF( norm_lsf_fx[i] == last_bin )
        {
            L_tmp = L_add(Bin_Ener_fx[last_bin-1], 0);
        }
        ELSE
        {
            L_tmp = L_max(Bin_Ener_fx[norm_lsf_fx[i]], Bin_Ener_fx[norm_lsf_fx[i]-1]); /* Q_ener */
            L_tmp = L_max(Bin_Ener_fx[norm_lsf_fx[i]+1], L_tmp); /* Q_ener */
        }

        IF (L_sub(L_tmp, MIN_LOG_FX) <= 0)
        {
            w_fft_fx[i] = MIN_LOG_VAL_FX;
            move16(); /* Q8 */
        }
        ELSE
        {
            exp = norm_l(L_tmp);
            move16();
            IF (L_tmp==0)
            {
                frac = 0;
                move16();
            }
            ELSE
            {
                frac = Log2_norm_lc(L_shl(L_tmp, exp));
            }
            exp = sub(sub(30,exp),Q_ener);
            L_tmp = Mpy_32_16(exp, frac, 24660); /* Q14 */ /* 10*log10(2) in Q13*/
            w_fft_fx[i] = round_fx(L_shl(L_tmp,10)); /* Q8 */
        }

        if (sub(w_fft_fx[i], min_fx) < 0)
        {
            min_fx = w_fft_fx[i];
            move16();
        }
    }

    FOR ( i=0; i<M; i++ )
    {
        IF( sub(w_fft_fx[i], min_fx) == 0)
        {
            w_fft_fx[i] = 2048;
            move16(); /* 2.0 in Q10 */
        }
        ELSE
        {
            L_tmp = L_shl( L_deposit_l(sub(w_fft_fx[i],min_fx)), 13); /* Q21 */
            exp = norm_l(L_tmp);
            frac = round_fx(L_shl(L_tmp,exp));
            exp = sub(add(exp,21),30);
            tmp_fx = div_s(16384,frac);
            L_tmp = Isqrt_lc(L_deposit_h(tmp_fx),&exp); /* Q(31-exp) */
            w_fft_fx[i] = round_fx(L_shl(L_tmp,sub(exp,5))); /* Q10 */
            w_fft_fx[i] = add(w_fft_fx[i],2048);
            move16(); /* Q10; 2.0 in Q10 */
        }
        w_fft_fx[i] = round_fx(Mult_32_16(Freq_w_Table_fx[norm_lsf_fx[i]], w_fft_fx[i])); /* Q10 */
    }

    /* 2) IHM weights*/
    FOR (i = 0; i < order; i++)
    {
        /* 2) IHM weights*/
        tmp1_fx = lsf_fx[i];
        move16();
        if (i > 0)
        {
            tmp1_fx = sub(tmp1_fx, lsf_fx[i-1]);
        }

        tmp2_fx = nf_fx;
        move16();
        if (sub(i, order - 1) != 0)
        {
            tmp2_fx = lsf_fx[i+1];
            move16();
        }
        tmp2_fx = sub(tmp2_fx, lsf_fx[i]);

        s1 = 15;
        move16();
        s2 = 15;
        move16();
        if(tmp1_fx == 0)
        {
            tmp1_fx = 8;
        }
        tmp1_fx = Inv16(tmp1_fx, &s1);
        if(tmp2_fx == 0)
        {
            tmp2_fx = 8;
        }
        tmp2_fx = Inv16(tmp2_fx, &s2);
        s1 = BASOP_Util_Add_MantExp(tmp1_fx, s1, tmp2_fx, s2, &tmp1_fx); /* x * 2.56 / pow(2.0, 15 + |s1|)   */
        tmp_fx = mult_r(nf_fx, 10430);
        s2 = norm_s(tmp_fx);
        tmp_fx = shl(tmp_fx, s2);
        s1 = sub(s1, s2);

        tmp1_fx = mult_r(tmp1_fx, tmp_fx); /* |s1| */
        s1 = abs_s(s1);

        /* 3) Fitting model combining the two weights*/
        L_tmp = L_add(ptr_lsf_fit_model[0][i], 0);  /* Q10 */
        L_tmp = L_add(L_tmp, L_shl(L_mult0(ptr_lsf_fit_model[1][i], tmp1_fx), sub(-5, s1))); /* Q10 */
        L_tmp = L_add(L_tmp, L_shl(L_mult0(mult_r(tmp1_fx, tmp1_fx), ptr_lsf_fit_model[2][i]), sub(7, shl(s1, 1))));
        L_tmp = L_add(L_tmp, L_shl(L_mult0(w_fft_fx[i], ptr_lsf_fit_model[3][i]), -12));
        move16(); /* Q10 */

        IF ( L_sub( L_shl( L_tmp, 5), InvIntTable[i+1] ) < 0 )
        {
            w_fx[i] = shr( InvIntTable[i+1], 7);
            move16();
        }
        ELSE
        {
            IF( norm_l( L_tmp ) < 14 )
            {
                w_fx[i] = MAX_16;
                move16();
            }
            ELSE
            {
                w_fx[i] = extract_l( L_shr( L_tmp, 2 ) );
            }
        }
    }

    return;
}
