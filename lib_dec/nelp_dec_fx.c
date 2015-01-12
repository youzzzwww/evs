/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches */
#include "prot_fx.h"       /* Function prototypes */

#include "rom_com_fx.h"
#include "stl.h"


/*===================================================================*/
/* FUNCTION      :  nelp_decoder_fx()                                */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  NELP decoding for the current frame              */
/*                                                                   */
/*-------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS  :                                         */
/*    _ (Struct)   st                                                */
/*    _ (Word16[]) exc_nelp    : adapt. excitation/total exc (Q0)    */
/*    _ (Word16[]) exc         : adapt. excitation exc (Q0)          */
/*    _ (Word16)   bfi         : frame error rate                    */
/*    _ (Word16)   coder_type  : coding type                         */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*    _ (Word16[]) exc_nelp    : adapt. excitation/total exc (Q0)    */
/*-------------------------------------------------------------------*/

/*    _ (Word16[])  shape1_filt_mem_dec : filter memory (Q0)         */
/*    _ (Word16[])  shape2_filt_mem_dec : filter memory (Q0)         */
/*    _ (Word16[])  shape3_filt_mem_dec : filter memory (Q0)         */
/*    _ (Word16[])  bp1_filt_mem_wb_dec : filter memory (Q0)         */
/*    _ (Word16[])  bp1_filt_mem_nb_dec : filter memory (Q0)         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*     _ None                                                        */
/*===================================================================*/

void nelp_decoder_fx( Decoder_State_fx *st_fx, Word16 *exc_nelp, Word16 *exc, Word16 *Q_exc, Word16 bfi, const Word16 coder_type
                      , Word16 *gain_buf
                    )
{
    Word16 i, fid = 0;
    Word16 ptr[L_FRAME], filtRes[L_FRAME], gain_fac; /*ptr, filtRes - Q0, gain_fac - Q14 */
    Word16 Gains[10];                 /* Q0 */
    Word32 Gain, E3, E2;
    Word16 BP1_ORDER = 4;
    Word16 ptr_tmp[L_FRAME];          /* Q0 */
    Word16 iG1, iG2[2];
    Word16 exp_E2, exp_E3, frac_E2, frac_E3;
    Word16 tmp, scale, exp, frac;
    Word32 L_tmp, L_tmp1;
    Word32 exc_sqr[L_SUBFR];
    Word32 max_exc_sqr;
    Word16 n;
    Word16 max_val = 0, norm_val = 0;

    test();
    test();
    test();
    IF ( sub(coder_type,UNVOICED) == 0 && sub(st_fx->bwidth_fx,NB) == 0 )
    {
        IF (sub(st_fx->last_nelp_mode_dec_fx,1) != 0)
        {
            BP1_ORDER = 7;
            move16();

            set32_fx(st_fx->bp1_filt_mem_nb_dec_fx, 0, BP1_ORDER*2);
        }
    }
    ELSE IF ( sub(coder_type,UNVOICED) == 0 && (sub(st_fx->bwidth_fx,WB) == 0|| sub(st_fx->bwidth_fx, SWB) == 0))
    {
        BP1_ORDER =4;
        move16();
        IF (sub(st_fx->last_nelp_mode_dec_fx,1) != 0)
        {
            set16_fx(st_fx->bp1_filt_mem_wb_dec_fx, 0 , BP1_ORDER*2);
        }
    }

    IF (sub(st_fx->last_nelp_mode_dec_fx,1) != 0)
    {
        set16_fx(st_fx->shape1_filt_mem_dec_fx, 0, 20);
        set16_fx(st_fx->shape2_filt_mem_dec_fx, 0, 20);
        set16_fx(st_fx->shape3_filt_mem_dec_fx, 0, 20);
    }

    IF (bfi == 0)
    {
        test();
        IF(sub(st_fx->rf_frame_type,RF_NELP) == 0 && sub(st_fx->use_partial_copy,1)==0)
        {
            iG1 = st_fx->rf_indx_nelp_iG1[0];
            iG2[0] = st_fx->rf_indx_nelp_iG2[0][0];
            iG2[1] = st_fx->rf_indx_nelp_iG2[0][1];
        }
        ELSE
        {
            /* Do Unvoiced/NELP Decoding */
            iG1    =(Word16) get_next_indice_fx( st_fx, 5 );
            move16();
            iG2[0] =(Word16) get_next_indice_fx( st_fx, 6 );
            move16();
            iG2[1] =(Word16) get_next_indice_fx( st_fx, 6 );
            move16();
        }

        test();
        test();
        IF ( sub(coder_type,UNVOICED) == 0 && (sub(st_fx->bwidth_fx,WB) == 0|| sub(st_fx->bwidth_fx, SWB) == 0))
        {
            test();
            IF(sub(st_fx->rf_frame_type,RF_NELP) == 0 && sub(st_fx->use_partial_copy,1)==0)
            {
                fid = st_fx->rf_indx_nelp_fid[0];
            }
            ELSE
            {
                fid = (Word16)get_next_indice_fx( st_fx, 2 );
                move16();
            }
        }

        *Q_exc = dequantize_uvg_fx(iG1, iG2, Gains, st_fx->bwidth_fx ,1 );
        move16();/* Gains - Q0/Q3 */
    }
    ELSE
    {
        FOR (i=1; i<=L_SUBFR; i++)
        {
            exc_sqr[i-1] = L_mult0(exc[-i],exc[-i]);
            move32();/*2*Q_exc */
        }
        max_exc_sqr = L_deposit_l(0);
        FOR (i=0; i<L_SUBFR; i++)
        {
            max_exc_sqr = L_max(max_exc_sqr,exc_sqr[i]);
        }
        IF(max_exc_sqr != 0)
        {
            exp = norm_l(max_exc_sqr);
            FOR (i=0; i<L_SUBFR; i++)
            {
                exc_sqr[i] = L_shl(exc_sqr[i],exp);
                move32();/*Q30 */
            }
            exp = sub(30,add(exp,add(*Q_exc,*Q_exc)));

            Gain = L_deposit_l(1);
            FOR (i=0 ; i<L_SUBFR; i++)
            {
                Gain = L_add(Gain,L_shr(exc_sqr[i],6)); /*Q24 */
            }
            Gain = Mult_32_16(Gain,20972); /*Q24; 20972=sqr(0.8) in Q15 */
            exp = sub(exp,6); /*due to /L_SUBFR */

            IF(s_and(exp,1) != 0)
            {
                Gain = L_shr(Gain,1); /*Q24 */
                exp = add(exp,1);
            }
            exp = shr(exp,1);
            n = norm_l(Gain);
            frac = round_fx(L_shl(Gain,n));
            n = sub(add(n,24),30);
            frac = div_s(16384,frac);
            Gain = Isqrt_lc(L_deposit_h(frac),&n); /*Q(31-n-exp) */
            tmp = round_fx(L_shl(Gain,sub(n+exp,15))); /*Q0 */
        }
        ELSE
        {
            tmp = 0;
            move16();
        }

        set16_fx(Gains, tmp, 10);
        *Q_exc = 0;
        move16();
    }


    gain_fac = 22446;
    move16();/* 1.37f - Q14 */
    test();
    test();
    if ( sub(coder_type,UNVOICED) == 0 && (sub(st_fx->bwidth_fx,WB) == 0|| sub(st_fx->bwidth_fx, SWB) == 0))
    {
        gain_fac = 19005;
        move16(); /* 1.16f - Q14 */
    }

    generate_nelp_excitation_fx(&(st_fx->nelp_dec_seed_fx), Gains, ptr, gain_fac);

    test();
    test();
    IF ( sub(coder_type,UNVOICED ) == 0&& (sub(st_fx->bwidth_fx,WB) == 0|| sub(st_fx->bwidth_fx, SWB) == 0) )
    {
        BP1_ORDER =4;
        move16();
        pz_filter_sp_fx(bp1_num_coef_wb_fx, bp1_den_coef_wb_fx, ptr, ptr_tmp, st_fx->bp1_filt_mem_wb_dec_fx,
                        BP1_ORDER, BP1_ORDER, L_FRAME, (sub(16,BP1_COEF_WB_QF)));

        Copy(ptr_tmp,ptr,L_FRAME); /*Q_exc */
    }

    test();
    IF ( sub(coder_type,UNVOICED ) == 0&& (sub(st_fx->bwidth_fx,NB) == 0) )
    {
        BP1_ORDER = 7;
        move16();

        FOR (i=0; i < L_FRAME; i++ )
        {
            max_val = s_max( ptr[i], max_val );
        }
        tmp = shl(BP1_ORDER,1);
        FOR (i=0; i < tmp; i++ )
        {
            max_val = s_max( round_fx( L_shr( st_fx->bp1_filt_mem_nb_dec_fx[i], 16)), max_val );
        }
        norm_val = norm_s(max_val);

        norm_val = s_max (0, sub(norm_val, 4)); /* 4 bit head room */

        norm_val = s_min ( norm_val,8); /* scale only for very low level signals */

        IF ( norm_val > 0 )
        {
            Scale_sig32(st_fx->bp1_filt_mem_nb_dec_fx, shl(BP1_ORDER,1), norm_val) ;
            Scale_sig(ptr, L_FRAME, norm_val);
            *Q_exc = add( norm_val, *Q_exc );
        }

        BP1_ORDER = 7;
        move16();
        pz_filter_dp_fx(bp1_num_coef_nb_fx_order7, bp1_den_coef_nb_fx_order7, ptr, ptr_tmp, st_fx->bp1_filt_mem_nb_dec_fx,
                        BP1_ORDER, BP1_ORDER, L_FRAME, (sub(16,BP1_COEF_NB_QF_ORDER7)));


        IF ( norm_val > 0 )
        {
            Scale_sig32(st_fx->bp1_filt_mem_nb_dec_fx, shl(BP1_ORDER,1), -norm_val) ;

        }

        Copy(ptr_tmp,ptr,L_FRAME); /*Q_exc */
    }

    E3 = L_deposit_l(1);
    FOR (i=0 ; i<L_FRAME; i++)
    {
        E3 = L_mac0(E3,ptr[i],ptr[i]); /*2*Q_exc */
    }


    test();
    test();
    IF ( sub(coder_type,UNVOICED ) == 0&& (sub(st_fx->bwidth_fx,WB) == 0|| sub(st_fx->bwidth_fx, SWB) == 0) )
    {
        pz_filter_sp_fx(shape1_num_coef_fx, shape1_den_coef_fx, ptr, ptr_tmp, st_fx->shape1_filt_mem_dec_fx,
                        10, 10, L_FRAME, (sub(16,SHAPE1_COEF_QF)));
        Copy(ptr_tmp,ptr,L_FRAME); /*Q_exc */

        SWITCH(fid)
        {
        case 1:
            /* Update other filter memory */
            pz_filter_sp_fx(shape3_num_coef_fx, shape3_den_coef_fx, ptr, filtRes, st_fx->shape3_filt_mem_dec_fx,
                            10, 10, L_FRAME, (sub(16,SHAPE3_COEF_QF)));

            /* filter the residual to desired shape */
            pz_filter_sp_fx(shape2_num_coef_fx, shape2_den_coef_fx, ptr, ptr_tmp, st_fx->shape2_filt_mem_dec_fx,
                            10, 10, L_FRAME, (sub(16,SHAPE2_COEF_QF)));

            Copy(ptr_tmp,ptr,L_FRAME); /*Q_exc */

            BREAK;
        case 2:
            /* Update other filter memory */
            pz_filter_sp_fx(shape2_num_coef_fx, shape2_den_coef_fx, ptr, filtRes, st_fx->shape2_filt_mem_dec_fx,
                            10, 10, L_FRAME, (sub(16,SHAPE2_COEF_QF)));

            /* filter the residual to desired shape */
            pz_filter_sp_fx(shape3_num_coef_fx, shape3_den_coef_fx, ptr, ptr_tmp, st_fx->shape3_filt_mem_dec_fx,
                            10, 10, L_FRAME, (sub(16,SHAPE3_COEF_QF)));

            Copy(ptr_tmp,ptr,L_FRAME); /*Q_exc */

            BREAK;
        default:
            /* Update other filter memory */
            pz_filter_sp_fx(shape2_num_coef_fx, shape2_den_coef_fx, ptr, filtRes, st_fx->shape2_filt_mem_dec_fx,
                            10, 10, L_FRAME, (sub(16,SHAPE2_COEF_QF)));
            pz_filter_sp_fx(shape3_num_coef_fx, shape3_den_coef_fx, ptr, filtRes, st_fx->shape3_filt_mem_dec_fx,
                            10, 10, L_FRAME, (sub(16,SHAPE3_COEF_QF)));

            BREAK;
        }

        E2 = L_deposit_l(1);
        FOR (i=0 ; i<L_FRAME; i++)
        {
            E2 = L_mac0(E2,ptr[i],ptr[i]); /*2*Q_exc */
        }

        exp_E3 = norm_l(E3);
        frac_E3 = extract_h(L_shl(E3,exp_E3));
        exp_E3 = sub(30,add(exp_E3,2*(*Q_exc)));

        exp_E2 = norm_l(E2);
        frac_E2 = round_fx(L_shl(E2,exp_E2));
        exp_E2 = sub(30,add(exp_E2,2*(*Q_exc)));

        scale = shr(sub(frac_E3,frac_E2),15);
        frac_E2 = shl(frac_E2,scale);
        exp_E2 = sub(exp_E2,scale);

        tmp = div_s(frac_E2,frac_E3);
        exp = sub(exp_E2,exp_E3);

        L_tmp1 = Isqrt_lc(L_deposit_h(tmp),&exp); /*Q(31-exp) */

        FOR (i=0; i<L_FRAME; i++)
        {
            L_tmp = Mult_32_16(L_tmp1,ptr[i]); /*Q(16-exp+Q_exc) */
            ptr[i] = round_fx(L_shl(L_tmp,exp)); /*Q_exc */
        }
    }

    Copy(ptr, exc_nelp, L_FRAME); /*Q_exc */
    set16_fx(gain_buf, 0, NB_SUBFR16k);
    return;
}


