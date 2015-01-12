/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "stl.h"
#include "prot_fx.h"
#include "stat_dec_fx.h"
#include "basop_util.h"


/************************************************************/
/*       Macro for the functions to be implemented.         */
/************************************************************/

void coderLookAheadInnovation(
    Word16 A_3Q12[],                /* input: coefficients NxAz[M+1]    */
    Word16 *pT,                     /* out:   pitch                     */
    HANDLE_PLC_ENC_EVS st,       /* i/o:   coder memory state        */
    Word16 *speechLookAhead_Qx,     /* i:   input speech in Q(st->Qold) */
    Word16 *old_exc,                /* i:   input excitation in Q(st->Qold) */
    Word16 L_frame
)
{
    Word16 i;
    Word16 prev_pitch, T0_fx;
    Word16 *exc_Qx, exc_buf_Qx[L_EXC_MEM+2*L_SUBFR+8];
    Word32 mantissa_max = -0x7fffffffL;
    Word16 subfr_len = 0;
    Word16 search_range = 8;
    Word16 exc_max;
    Word16 exc_sh;
    Word32 ps,alp, alp_ini;
    Word32 alp_s1, alp_s2;
    Word16 k;
    Word16 ps_e,alp_e;
    Word32 max_ps, max_ps_tmp;
    Word16 max_ps_e;
    Word16 tmp_loop;

    /* Debug init (not instrumented) */
    T0_fx = -3000;
    subfr_len = shl(L_SUBFR,1); /* 2*L_SUBFR */
    if( sub( L_FRAME16k, L_frame ) > 0 )
    {
        subfr_len = add(L_SUBFR,48); /* 1.75*L_SUBFR */
    }

    /*------------------------------------------------------------------------*
     * - BASOP specific initialization.                                       *
     *------------------------------------------------------------------------*/
    /* initialization */
    exc_Qx = exc_buf_Qx + L_EXC_MEM + 8;
    FOR( i=0; i<L_EXC_MEM+8; i++ )
    {
        exc_buf_Qx[i] = old_exc[i];
        move16();
    }


    /*------------------------------------------------------------------------*
     * - Get residual signal and target at lookahead part.                    *
     *------------------------------------------------------------------------*/
    /* find LP residual signal for look-ahead part */
    getLookAheadResSig( speechLookAhead_Qx, A_3Q12, exc_Qx, L_frame, 2 );
    Scale_sig( exc_Qx, subfr_len, 1 );

    /* find target signals */
    prev_pitch = st->T0_4th;
    move16();
    /* find best candidate of pitch lag */
    T0_fx = st->T0_4th;
    move16();
    mantissa_max = -0x7fffffffL;
    move32();
    max_ps = -0x7fffffffL;
    move32();
    max_ps_e = 16;
    move16();

    /*find maximum*/
    exc_max = 0;
    move16();
    tmp_loop = s_min(-prev_pitch+search_range+subfr_len,0);
    FOR(i=-prev_pitch-search_range; i < tmp_loop; i++)
    {
        exc_max = s_max(exc_Qx[i],exc_max);
    }
    FOR(i= 0; i<subfr_len; i++)
    {
        exc_max = s_max(exc_max,exc_Qx[i]);
    }
    /*calculate scaling factor for optimal precision and assure no overflow in dotproduct*/
    exc_sh = sub(15,norm_s(sub(subfr_len,1))); /*ceil(ld(subfr_len))*/
    exc_sh = s_max(sub(exc_sh,norm_s(s_max(abs_s(exc_max),1))),0);
    exc_sh = shr(add(exc_sh,1),1);

    /*scale buffer only where its needed*/
    tmp_loop = s_min(-prev_pitch+search_range+subfr_len,0);
    FOR(i=-prev_pitch-search_range; i < tmp_loop; i++)
    {
        exc_Qx[i] = shr(exc_Qx[i],exc_sh);
        move16();
    }
    FOR(i= 0; i<subfr_len; i++)
    {
        exc_Qx[i] = shr(exc_Qx[i],exc_sh);
        move16();
    }





    /*Calculate "big" dotproduct from buffer, including search range*/
    alp_ini   = L_deposit_l(0);
    move16();
    FOR(i=-prev_pitch-search_range; i< -prev_pitch+search_range+subfr_len; i++)
    {
        alp_ini = L_mac(alp_ini,exc_Qx[i],exc_Qx[i]);
    }


    FOR( i=-search_range; i<search_range; i++ )
    {
        test();
        test();
        test();
        test();
        test();
        IF( ( sub(L_frame,L_FRAME16k)<0 && ( sub(add(prev_pitch,i),PIT_MAX)>0 || sub(add(prev_pitch,i),PIT_MIN)<0 ) )
            || ( sub(L_frame,L_FRAME16k)==0 && ( sub(add(prev_pitch,i),PIT16k_MAX)>0 || sub(add(prev_pitch,i),PIT16k_MIN)<0 ) )
          )
        {
            CONTINUE;
        }
        ps        = L_deposit_l(0);
        alp_s1    = L_deposit_l(0);
        alp_s2    = L_deposit_l(0);

        FOR(k=0; k<subfr_len; k++)
        {
            ps=L_mac(ps,exc_Qx[k],exc_Qx[k-prev_pitch-i]);
        }

        /*calculate "small" dotproducts in order to subtract them from the "bigger" one*/
        FOR(k=negate(add(prev_pitch,search_range)); k<-prev_pitch-i; k++)
        {
            alp_s1 = L_mac(alp_s1,exc_Qx[k],exc_Qx[k]);
        }
        tmp_loop =  sub(add(search_range,subfr_len)  ,prev_pitch);
        FOR(k =  + subfr_len - i -prev_pitch ; k < tmp_loop; k++)
        {
            alp_s2 = L_mac(alp_s2,exc_Qx[k],exc_Qx[k]);
        }
        alp = L_sub(alp_ini,L_add(alp_s1,alp_s2));
        alp = L_max(alp, 1);  /* alp must not be 0 */
        alp_e = shl(exc_sh,1);
        ps_e = shl(exc_sh,1);
        alp = ISqrt32(alp, &alp_e);
        ps = Mpy_32_16_1(ps,round_fx(alp)); /*alp_e+ps_e*/

        ps_e = add(alp_e,ps_e);

        BASOP_SATURATE_WARNING_OFF
        max_ps_tmp = L_shl(max_ps,sub(max_ps_e,ps_e));
        BASOP_SATURATE_WARNING_ON


        IF (L_sub(max_ps_tmp , ps) < 0)
        {
            max_ps = L_add(ps, 0);
            max_ps_e = ps_e;
            move16();
            T0_fx = add(prev_pitch,i);
        }

    }
    mantissa_max = max_ps;
    move32();
    if( mantissa_max < 0 )
    {
        T0_fx = st->T0_4th;
        move16();
    }

    /* Update excitation */
    pT[0] = T0_fx;
    move16();

    return;
}

/************************************************************/
/*                     Static functions                     */
/************************************************************/
void enc_prm_side_Info( HANDLE_PLC_ENC_EVS hPlc_Ext, Encoder_State_fx *st )
{
    Word16 diff_pitch;
    Word16 bits_per_subfr, search_range;

    bits_per_subfr = 4;
    move16();
    search_range = 8;
    move16();

    IF( sub(hPlc_Ext->nBits,1)>0 )
    {

        push_next_indice_fx(st, 1, 1);

        diff_pitch = sub(hPlc_Ext->T0, hPlc_Ext->T0_4th);
        test();
        if( (sub(diff_pitch,sub(search_range,1)) > 0) || (sub(diff_pitch,-search_range) < 0) )
        {
            diff_pitch = 0;
            move16();
        }

        push_next_indice_fx(st, add(diff_pitch, search_range), bits_per_subfr);
    }
    ELSE
    {
        push_next_indice_fx(st, 0, 1);
    }


    return;
}

/************************************************************/
/*       Functions for encoder side loss simulation         */
/************************************************************/
void encoderSideLossSimulation(
    Encoder_State_fx *st,
    HANDLE_PLC_ENC_EVS hPlc_Ext,
    Word16 *lsf_q,                  /* Q1*1.28 */
    Word16 stab_fac,                 /* Q15 */
    Word8 calcOnlyISF,
    const Word16 L_frame
)
{
    Word16 lspLocal_Q15[M];
    Word16 const* xsfBase;                      /* base for differential XSF coding */


    /*************************************************************
     * Decoder state could be stored with memcpy,
     * since Decoder_State does not contain pointer member.
     *************************************************************/

    /* Decoder State Update */
    IF( sub(L_frame,L_FRAME_16k)==0 )
    {
        lsf2lsp_fx( lsf_q, lspLocal_Q15, M, INT_FS_16k_FX );
    }
    ELSE
    {
        lsf2lsp_fx( lsf_q, lspLocal_Q15, M, INT_FS_FX );
    }


    xsfBase = PlcGetLsfBase (st->lpcQuantization,
                             st->narrowBand,
                             st->sr_core);

    Copy( st->mem_MA_fx, hPlc_Ext->mem_MA_14Q1, M );
    Copy( st->mem_AR_fx, hPlc_Ext->mem_AR, M );


    /* ISF parameter processing for concealment */
    updateLSFForConcealment( hPlc_Ext, lsf_q, M );
    hPlc_Ext->stab_fac_Q15 = stab_fac;
    move16();

    Copy( lsf_q, hPlc_Ext->lsfold_14Q1, M );
    Copy( lspLocal_Q15, hPlc_Ext->lspold_Q15, M );


    IF (calcOnlyISF != 0)
    {
        /* ISF concealment simulation */
        getConcealedLSF( hPlc_Ext, xsfBase, st->clas_fx, L_frame );
    }
    ELSE
    {
        Word16 old_exc_Qx[L_EXC_MEM+8];
        Word16 A_3Q12[(NB_SUBFR16k+1)*(M+1)];
        Word16 *speechLookAhead_Qx;

        /* calculate Q-value for input speech */
        speechLookAhead_Qx = &(st->speech_enc_pe[L_frame]);

        Copy( hPlc_Ext->old_exc_Qold, old_exc_Qx, 8 );
        Copy( hPlc_Ext->LPDmem->old_exc, &old_exc_Qx[8], L_EXC_MEM );
        Scale_sig( old_exc_Qx, 8, hPlc_Ext->Q_exp );

        /* ISF concealment simulation */
        getConcealedLP( hPlc_Ext, A_3Q12, xsfBase, st->clas_fx, L_frame );

        /* apply encoder side PLC simulation */

        coderLookAheadInnovation( A_3Q12, &(hPlc_Ext->T0), hPlc_Ext, speechLookAhead_Qx, old_exc_Qx, L_frame );
    }
    return;
}


void GplcTcxEncSetup( Encoder_State_fx *st, HANDLE_PLC_ENC_EVS hPlc_Ext, Word16 Q_new )
{
    hPlc_Ext->T0_4th = st->tcxltp_pitch_int;
    move16();
    hPlc_Ext->Q_exp = sub( Q_new, hPlc_Ext->Q_new);
    move16();
    hPlc_Ext->Q_new = Q_new;
    move16();
    set16_fx( hPlc_Ext->old_exc_Qold, 0, 8);
}

Word16 encSideSpecPowDiffuseDetector(
    Word16 *lsf_ref,
    Word16 *lsf_con,
    Word32 sr_core,
    Word16 *prev_lsf4_mean,
    Word8 sw
)
{
    Word16 tmp;
    Word16 lsf_mod[M];
    Word32 dist1, dist2, cum_dist1, cum_dist2;
    Word16 lsf4_mean;
    Word16 th;
    Word16 idx;
    Word16 cnt_imprv, i;
    Word32 L_tmp;
    Word16 th_dif;

    /* calculate the mean of the lowest 4 LSFs */

    L_tmp = L_mult(lsf_ref[0], FL2WORD16(1.0/4.0));
    L_tmp = L_mac(L_tmp, lsf_ref[1], FL2WORD16(1.0/4.0));
    L_tmp = L_mac(L_tmp, lsf_ref[2], FL2WORD16(1.0/4.0));
    lsf4_mean = mac_r(L_tmp, lsf_ref[3], FL2WORD16(1.0/4.0));

    IF(sw)
    {
        Copy(lsf_con, lsf_mod, M);

        modify_lsf(lsf_mod, M, sr_core );

        move16();
        move16();
        cum_dist1 = 0;
        cum_dist2 = 0;

        cnt_imprv = 0;

        IF( L_sub( sr_core, 16000 ) == 0 )
        {
            th = 2560;
            move16(); /* LSF */
            th_dif = 288;
            move16(); /* LSF */
        }
        ELSE
        {
            th = 2048;
            move16(); /* LSF */
            th_dif = 230;
            move16(); /* LSF */
        }

        FOR(i = 0; i < M; i++)
        {
            tmp = sub(lsf_con[i], lsf_ref[i]);
            dist1 = L_mult(tmp, tmp);
            tmp = sub(lsf_mod[i], lsf_ref[i]);
            dist2 = L_mult(tmp, tmp);

            if(L_sub(dist1, dist2) > 0)
            {
                cnt_imprv = add(cnt_imprv, 1);
            }
            cum_dist1 = L_add(cum_dist1, dist1);
            cum_dist2 = L_add(cum_dist2, dist2);
        }

        idx = 0;
        move16();

        test();
        test();
        test();
        if(L_sub(cum_dist1, L_add(cum_dist2, Mpy_32_16_1(cum_dist2, 4915))) > 0
                && sub(sub(lsf4_mean, *prev_lsf4_mean), th_dif) > 0
                && sub(*prev_lsf4_mean, th) < 0
                && sub(cnt_imprv, 2) > 0 )
        {
            idx = 1;
            move16();
        }

    }
    ELSE
    {
        move16();
        idx = 0;
    }
    /* update parameters */
    move16();
    *prev_lsf4_mean = lsf4_mean;

    return idx;
}

void updateSpecPowDiffuseIdx( Encoder_State_fx *st)
{
    Word16 min_gp;
    Word16 k;


    move32();
    move16();
    st->mean_gc[1] = st->gain_code[0];
    min_gp = st->bpf_gainT[0];

    FOR(k = 1; k < 4; k++)
    {
        st->mean_gc[1] = L_add(st->mean_gc[1], st->gain_code[k]);
        min_gp = s_min(min_gp, st->bpf_gainT[k]);
    }

    /* Suppress saturation warning in threshold comparison. */
    test();
    if(L_sub(st->mean_gc[1], L_add(st->mean_gc[0], Mpy_32_16_r(st->mean_gc[0], FL2WORD16_SCALE(0.098, 0)))) < 0 ||
            sub(min_gp, FL2WORD16_SCALE(0.82, 1)) > 0)
    {
        move16();
        st->glr_idx [0]= 0;
    }
    move16();
    st->mean_gc[0] = st->mean_gc[1];

}

