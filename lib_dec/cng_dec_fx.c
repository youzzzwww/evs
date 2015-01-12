/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */

#include "stl.h"
#include "prot_fx.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "basop_mpy.h"
#include <assert.h>

/*Temporary location to be move in prot* when merge is done*/
void E_LPC_f_isp_a_conversion(const Word16 *isp, Word16 *a, const Word16 m);
void E_LPC_f_lsp_a_conversion(const Word16 *isp, Word16 *a, const Word16 m);

/*-----------------------------------------------------------------*
 * Local functions
 *-----------------------------------------------------------------*/

static void shb_CNG_decod_fx( Decoder_State_fx *st_fx, const Word16 *synth_fx, Word16 *shb_synth_fx, const Word16 sid_bw
                              ,const Word16 Qsyn
                            );

/*-----------------------------------------------------------------*
 * Decode residual signal energy
 *-----------------------------------------------------------------*/

void CNG_dec_fx(
    Decoder_State_fx *st_fx,          /* i/o: State structure                          */
    const Word16 L_frame,           /* i  : length of the frame                 Q0   */
    Word16 Aq[],              /* o  : LP coefficients                     Q12  */
    const Word32  core_brate,       /* i  : core bitrate                        Q0   */
    Word16 *lsp_new,          /* i/o: current frame LSPs                  Q15  */
    Word16 *lsf_new,          /* i/o: current frame LSFs                  Qlog2(2.56) */
    Word16 *allow_cn_step,    /* o  : allow CN step                       Q0   */
    Word16 *sid_bw            /* i  : 0-NB/WB, 1-SWB SID                       */
    ,Word32 *q_env
)
{
    Word16 istep;
    Word16 i, L_enr_index;
    Word32 L_ener;
    Word16 ener_fra,ener_int;
    Word16 num_bits;
    Word16 weights, ptr, j, k;
    Word16 m = 0;
    Word16 tmp[HO_HIST_SIZE*M];
    Word16 burst_ho_cnt = 0;
    Word16 ll,s_ptr;
    Word32 L_enr,L_tmp1;
    Word16 tmp1,exp;
    Word16 lsf_tmp[M];
    Word32 C[M];
    Word32 max[2];
    Word16 max_idx[2];
    Word16 ftmp_fx;
    Word16 lsp_tmp[M];
    Word16 dev;
    Word16 max_dev;
    Word16 dist;
    Word16 tmpv;
    Word16 env_idx[2];
    Word32 enr1;
    Word32 env[NUM_ENV_CNG];
    Word32 tmp_env[HO_HIST_SIZE*NUM_ENV_CNG];
    Word32 L_tmp;
    Word16 fra;
    Word16 temp_lo_fx, temp_hi_fx;
    Word16 exp_pow;
    Word16 tmp_loop;

    m = 0;
    move16();
    /*-----------------------------------------------------------------*
     * Decode CNG spectral envelope (only in SID frame)
     *-----------------------------------------------------------------*/
    test();
    IF ( L_sub(core_brate,SID_1k75) == 0 || L_sub(core_brate, SID_2k40) == 0 )
    {
        /* de-quantize the LSF vector */
        IF ( st_fx->Opt_AMR_WB_fx != 0 )
        {
            /* Flt function */
            isf_dec_amr_wb_fx( st_fx, Aq, lsf_new, lsp_new);
        }
        ELSE
        {
            lsf_dec_fx( st_fx,
            0,
            L_frame, INACTIVE, -1, Aq, lsf_new, lsp_new, 0 );
        }
    }
    ELSE
    {
        /* Use old LSP vector */
        Copy( st_fx->lsp_old_fx, lsp_new, M );
        Copy( st_fx->lsf_old_fx, lsf_new, M );
    }

    /* Initialize the CNG spectral envelope in case of the very first CNG frame */
    IF( st_fx->first_CNG_fx == 0 )
    {
        Copy( st_fx->lsp_old_fx, st_fx->lspCNG_fx, M );
    }

    /*-----------------------------------------------------------------*
     * Decode residual signal energy
     *-----------------------------------------------------------------*/

    *allow_cn_step = 0;
    move16();
    test();
    IF( L_sub(core_brate,SID_1k75) == 0 || L_sub(core_brate, SID_2k40) == 0 )
    {
        istep  = ISTEP_AMR_WB_SID_FX;
        move16();
        if( L_sub(core_brate,SID_2k40) == 0 )
        {
            istep = ISTEP_SID_FX;
            move16();
        }

        /* intialize the energy quantization parameters */
        num_bits = 6;
        move16();
        if( st_fx->Opt_AMR_WB_fx == 0 )
        {
            num_bits = 7;
            move16();
        }

        /* decode the energy index */
        L_enr_index = get_next_indice_fx( st_fx, num_bits );

        test();
        test();
        test();
        if (L_sub(st_fx->last_core_brate_fx,SID_1k75) > 0 &&
                st_fx->first_CNG_fx != 0 &&
                st_fx->old_enr_index_fx >= 0 &&
                sub(L_enr_index, add(st_fx->old_enr_index_fx,1)) > 0)
        {
            *allow_cn_step = 1;
            move16();
        }

        st_fx->old_enr_index_fx = L_enr_index;
        move16();
        if ( L_enr_index == 0 )
        {
            L_enr_index = -5;
            move16();
        }
        /* st_fx->Enew = L_enr_index / step - 2.0f;*/
        L_ener = L_mult(L_enr_index, istep); /* Q16 (0+15) */
        /* substract by 2 not done to leave Energy in Q2 */

        /* extract integral and fractional parts */
        ener_fra = L_Extract_lc(L_ener, &ener_int);
        ener_int = add(ener_int, 4); /* Q2 to Q6 */

        /* find the new energy value */
        st_fx->Enew_fx = Pow2(ener_int, ener_fra);

        IF( L_sub(core_brate,SID_2k40) == 0 )
        {
            burst_ho_cnt = get_next_indice_fx( st_fx, 3 ); /* 3bit */

            *sid_bw = get_next_indice_fx( st_fx, 1 );
            IF ( *sid_bw == 0 )
            {
                env_idx[0] = get_next_indice_fx( st_fx, 6 );
                move16();

                /* get quantized res_env_details */
                FOR ( i=0; i<NUM_ENV_CNG; i++ )
                {
                    q_env[i] = CNG_details_codebook_fx[env_idx[0]][i];
                    move32();
                }
            }
        }
        /* Reset CNG history if CNG frame length is changed */
        test();
        test();
        if ( sub(st_fx->bwidth_fx,WB) == 0 && st_fx->first_CNG_fx != 0 && sub(st_fx->L_frame_fx,st_fx->last_CNG_L_frame_fx) != 0 )
        {
            st_fx->ho_hist_size_fx = 0;
            move16();
        }
    }

    /*---------------------------------------------------------------------*
     * CNG spectral envelope update
     * Find A(z) coefficients
     *---------------------------------------------------------------------*/
    test();
    test();
    test();
    IF( st_fx->last_core_brate_fx <= SID_2k40 )
    {
        /* Reset hangover counter if not first SID period */
        if( L_sub(st_fx->core_brate_fx,FRAME_NO_DATA) > 0 )
        {
            st_fx->num_ho_fx = 0;
            move16();
        }
        /* Update LSPs if last SID energy not outlier or insufficient number of hangover frames */
        test();
        IF( sub(st_fx->num_ho_fx,3) < 0 || L_sub(Mult_32_16(st_fx->Enew_fx,21845 /*1/1.5f, Q15*/), st_fx->lp_ener_fx) < 0 )
        {
            FOR( i=0; i<M; i++ )
            {
                /* AR low-pass filter  */
                st_fx->lspCNG_fx[i] = mac_r(L_mult(CNG_ISF_FACT_FX, st_fx->lspCNG_fx[i]), 32768-CNG_ISF_FACT_FX, lsp_new[i]);
                move16(); /* Q15 (15+15+1-16) */
            }
        }
    }
    ELSE
    {
        /* Update CNG_mode if allowed */
        test();
        test();
        test();
        IF( ( st_fx->Opt_AMR_WB_fx || sub(st_fx->bwidth_fx,WB) == 0 )
        && ( !st_fx->first_CNG_fx || sub(st_fx->act_cnt2_fx,MIN_ACT_CNG_UPD) >= 0 ) )
        {
            IF( L_sub(st_fx->last_active_brate_fx,ACELP_16k40)  > 0)
            {
                st_fx->CNG_mode_fx = -1;
                move16();
            }
            ELSE IF( L_sub(st_fx->last_active_brate_fx,ACELP_13k20) > 0 )
            {
                st_fx->CNG_mode_fx = 4;
                move16();
            }
            ELSE IF( L_sub(st_fx->last_active_brate_fx,ACELP_9k60) > 0 )
            {
                st_fx->CNG_mode_fx = 3;
                move16();
            }
            ELSE IF( L_sub(st_fx->last_active_brate_fx,ACELP_8k00) > 0 )
            {
                st_fx->CNG_mode_fx = 2;
                move16();
            }
            ELSE IF( L_sub(st_fx->last_active_brate_fx,ACELP_7k20) > 0 )
            {
                st_fx->CNG_mode_fx = 1;
                move16();
            }
            ELSE
            {
                st_fx->CNG_mode_fx = 0;
                move16();
            }
        }

        /* If first sid after active burst update LSF history from circ buffer */
        burst_ho_cnt = s_min(burst_ho_cnt, st_fx->ho_circ_size_fx); /* MODE1_DTX_IN_CODEC_B_FIX   */
        st_fx->act_cnt_fx = 0;
        move16();
        s_ptr = add(sub(st_fx->ho_circ_ptr_fx, burst_ho_cnt),1);
        move16();
        if( s_ptr < 0 )
        {
            s_ptr = add(s_ptr,st_fx->ho_circ_size_fx);
        }

        FOR( ll = burst_ho_cnt; ll > 0; ll-- )
        {
            st_fx->ho_hist_ptr_fx = add(st_fx->ho_hist_ptr_fx,1);
            if( sub(st_fx->ho_hist_ptr_fx, HO_HIST_SIZE) == 0 )
            {
                st_fx->ho_hist_ptr_fx = 0;
                move16();
            }

            /* Conversion between 12.8k and 16k LSPs */
            test();
            test();
            test();
            IF( ( sub(L_frame,L_FRAME16k) == 0 && st_fx->ho_16k_lsp_fx[s_ptr] == 0 ) || ( sub(L_frame,L_FRAME) == 0 && sub(st_fx->ho_16k_lsp_fx[s_ptr],1) == 0 ) )
            {
                /* Conversion from 16k LPSs to 12k8 */
                lsp_convert_poly_fx( &(st_fx->ho_lsp_circ_fx[s_ptr*M]), L_frame, 0 );
            }
            /* update the circular buffers */
            Copy(&(st_fx->ho_lsp_circ_fx[s_ptr*M]), &(st_fx->ho_lsp_hist_fx[st_fx->ho_hist_ptr_fx*M]), M );
            Copy32(&(st_fx->ho_ener_circ_fx[s_ptr]), &(st_fx->ho_ener_hist_fx[st_fx->ho_hist_ptr_fx]), 1 );
            Copy32(&(st_fx->ho_env_circ_fx[s_ptr*NUM_ENV_CNG]), &(st_fx->ho_env_hist_fx[st_fx->ho_hist_ptr_fx*NUM_ENV_CNG]), NUM_ENV_CNG );

            st_fx->ho_hist_size_fx = add(st_fx->ho_hist_size_fx,1);

            if (sub(st_fx->ho_hist_size_fx, HO_HIST_SIZE) > 0)
            {
                st_fx->ho_hist_size_fx = HO_HIST_SIZE;
                move16();
            }

            s_ptr = add(s_ptr,1);
            if( sub(s_ptr, st_fx->ho_circ_size_fx) == 0 )
            {
                s_ptr = 0;
                move16();
            }
        }

        IF( st_fx->ho_hist_size_fx > 0 )    /* can be -1 at init    MODE1_DTX_IN_CODEC_B_FIX    */
        {
            /* *allow_cn_step |= ( st_fx->ho_ener_hist[st_fx->ho_hist_ptr] > 4.0f * st_fx->lp_ener );*/
            L_tmp1 = L_shr(st_fx->ho_ener_hist_fx[st_fx->ho_hist_ptr_fx], 2);
            L_tmp1 = L_sub(L_tmp1, st_fx->lp_ener_fx);

            if(L_tmp1 > 0)
            {
                *allow_cn_step = s_or(*allow_cn_step,1);
            }
        }
        test();
        IF ( *allow_cn_step == 0 && st_fx->ho_hist_size_fx > 0 )
        {
            /* Use average of energies below last energy */
            ptr = st_fx->ho_hist_ptr_fx;
            move16();
            Copy( &(st_fx->ho_lsp_hist_fx[ptr*M]), tmp, M );
            Copy32( &st_fx->ho_env_hist_fx[ptr*NUM_ENV_CNG], tmp_env, NUM_ENV_CNG );

            L_enr = Mult_32_16(st_fx->ho_ener_hist_fx[ptr],W_DTX_HO_FX[0]) ;/* Q6+15-15->Q6 */

            weights = W_DTX_HO_FX[0]; /* Q15 */

            m = 1;
            move16();
            FOR( k=1; k<st_fx->ho_hist_size_fx; k++ )
            {
                ptr = sub(ptr,1);
                if( ptr < 0 )
                {
                    ptr = HO_HIST_SIZE - 1;
                    move16();
                }

                test();
                IF ( L_sub(Mult_32_16(st_fx->ho_ener_hist_fx[ptr],ONE_OVER_BUF_H_NRG_FX),st_fx->ho_ener_hist_fx[st_fx->ho_hist_ptr_fx]) < 0 &&
                     L_sub(st_fx->ho_ener_hist_fx[ptr],Mult_32_16(st_fx->ho_ener_hist_fx[st_fx->ho_hist_ptr_fx], BUF_L_NRG_FX)) > 0 )
                {
                    /*enr += W_DTX_HO[k] * st_fx->ho_ener_hist[ptr];*/
                    L_tmp1 = Mult_32_16(st_fx->ho_ener_hist_fx[ptr],W_DTX_HO_FX[k]) ; /* Q6+15-15->Q6 */
                    L_enr = L_add(L_enr,L_tmp1); /* Q6 */

                    /*weights += W_DTX_HO[k];*/
                    weights = add( weights, W_DTX_HO_FX[k]); /* Q15 */

                    Copy( &st_fx->ho_lsp_hist_fx[ptr*M], &tmp[m*M], M );
                    Copy32( &st_fx->ho_env_hist_fx[ptr*NUM_ENV_CNG], &tmp_env[m*NUM_ENV_CNG], NUM_ENV_CNG );
                    m = add(m,1);
                }
            }

            /*enr /= weights;*/
            exp = norm_s(weights);
            tmp1 = div_s(1<<(14-exp),weights); /* Q(15+14-exp-15) */
            L_tmp1 = Mult_32_16(L_enr,tmp1); /* Q(14-exp+6-15)->Q(5-exp) */
            L_enr = L_shl(L_tmp1,exp+1); /* Q6 */

            st_fx->lp_ener_fx = L_enr; /* Q6 */

            set32_fx( max, 0, 2 );
            set16_fx( max_idx, 0, 2 );

            FOR( i=0; i<m; i++ )
            {
                IF ( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
                {
                    lsp2lsf_fx( &tmp[i*M], lsf_tmp, M, INT_FS_FX );
                    ftmp_fx = 964;
                    move16();/*X2.56 */
                    tmpv = sub(16384,add(lsf_tmp[M-1],ftmp_fx)); /*QX2.56*/
                    L_tmp = L_mult0(tmpv,tmpv); /*QX6.5536*/
                }
                ELSE
                {
                    lsp2lsf_fx( &tmp[i*M], lsf_tmp, M, INT_FS_16k_FX );
                    ftmp_fx = 1205;
                    move16();/*QX2.56*/
                    tmpv = sub(20480,add(lsf_tmp[M-1],ftmp_fx)); /*QX2.56*/
                    L_tmp = L_mult0(tmpv,tmpv); /*QX6.5536*/
                }

                tmpv = sub(lsf_tmp[0],ftmp_fx); /*QX2.56*/
                L_tmp = L_mac0(L_tmp,tmpv,tmpv); /*QX6.5536*/
                FOR ( j=0; j<M-1; j++ )
                {
                    tmpv = sub(sub(lsf_tmp[j+1],lsf_tmp[j]),ftmp_fx); /*QX2.56*/
                    L_tmp = L_mac0(L_tmp,tmpv,tmpv); /*QX6.5536*/
                }

                C[i] = Mpy_32_16_1(L_tmp,1928); /*QX6.5536*/ move32();

                IF ( L_sub(C[i],max[0]) > 0 )
                {
                    max[1] = max[0];
                    move32();
                    max_idx[1] = max_idx[0];
                    move16();
                    max[0] = C[i];
                    move32();
                    max_idx[0] = i;
                    move16();
                }
                ELSE IF ( L_sub(C[i],max[1]) > 0 )
                {
                    max[1] = C[i];
                    move32();
                    max_idx[1] = i;
                    move16();
                }
            }

            IF ( sub(m,1) == 0 )
            {
                Copy(tmp, lsp_tmp, M);
            }
            ELSE IF ( sub(m,4) < 0 )
            {
                FOR ( i=0; i<M; i++ )
                {
                    L_tmp1 = 0;
                    FOR ( j=0; j<m; j++ )
                    {
                        L_tmp1 = L_add(L_tmp1,L_deposit_l(tmp[j*M+i]));
                    }

                    L_tmp1 = L_sub(L_tmp1,L_deposit_l(tmp[max_idx[0]*M+i]));
                    tmpv= div_s(1,sub(m,1)); /*Q15*/
                    L_tmp1 = Mpy_32_16_1(L_tmp1,tmpv); /*Q15*/
                    lsp_tmp[i] = extract_l(L_tmp1); /*Q15*/
                }
            }
            ELSE
            {
                FOR ( i=0; i<M; i++ )
                {
                    L_tmp1 = 0;
                    FOR ( j=0; j<m; j++ )
                    {
                        L_tmp1 = L_add(L_tmp1,L_deposit_l(tmp[j*M+i]));
                    }

                    L_tmp1 = L_sub(L_tmp1,L_add(L_deposit_l(tmp[max_idx[0]*M+i]),L_deposit_l(tmp[max_idx[1]*M+i]))); /*Q15*/
                    tmpv= div_s(1,sub(m,2)); /*Q15*/
                    L_tmp1 = Mpy_32_16_1(L_tmp1,tmpv); /*Q15*/
                    lsp_tmp[i] = extract_l(L_tmp1); /*Q15*/
                }
            }

            dist = 0; /*Q15*/
            max_dev = 0; /*Q15*/
            FOR ( i=0; i<M; i++ )
            {
                dev = abs_s(sub(lsp_tmp[i],lsp_new[i])); /*Q15*/
                dist = add(dist,dev); /*Q15*/
                if ( sub(dev,max_dev) > 0 )
                {
                    max_dev = dev;
                    move16();
                }
            }

            test();
            IF ( sub(dist,13107) > 0 || sub(max_dev,3277) > 0 )
            {
                FOR( i=0; i<M; i++ )
                {
                    st_fx->lspCNG_fx[i] = lsp_tmp[i];
                    move16(); /*Q15*/
                }
            }
            ELSE
            {
                FOR( i=0; i<M; i++ )
                {
                    /* AR low-pass filter  */
                    st_fx->lspCNG_fx[i] = add(mult_r(26214,lsp_tmp[i]),mult_r(6554,lsp_new[i]));
                }
            }
            FOR ( i=0; i<NUM_ENV_CNG; i++ )
            {
                env[i] = 0;
                FOR ( j=0; j<m; j++ )
                {
                    /* env[i] += tmp_env[j*NUM_ENV_CNG+i];*/
                    env[i] = L_add(env[i],tmp_env[j*NUM_ENV_CNG+i]);
                    move32();
                }
                /*    env[i] /= (float)m;  */
                /*    env[i] = env[i] - 2*st_fx->lp_ener_fx; */
                IF(sub(m,1) == 0)
                {
                    env[i] = L_sub(env[i],L_add(st_fx->lp_ener_fx,st_fx->lp_ener_fx));
                    move32();
                }
                ELSE
                {
                    tmp1 = div_s(1,m);
                    env[i] = Mult_32_16(env[i],tmp1);
                    env[i] = L_sub(env[i],L_add(st_fx->lp_ener_fx,st_fx->lp_ener_fx));
                    move32();
                }
            }

            Copy32(env, st_fx->lp_env_fx, NUM_ENV_CNG);
        }
        ELSE
        {
            Copy( lsp_new, st_fx->lspCNG_fx, M );  /* use newly analyzed ISFs */
        }
    }

    test();
    IF( L_sub(st_fx->core_brate_fx, SID_1k75) == 0 || L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
    {
        /* Update hangover memory during CNG */
        test();
        IF ( *allow_cn_step == 0 && L_sub(st_fx->Enew_fx,L_add(st_fx->lp_ener_fx,L_shr(st_fx->lp_ener_fx,1))) < 0)
        {
            /* update the pointer to circular buffer of old LSP vectors */
            st_fx->ho_hist_ptr_fx = add(st_fx->ho_hist_ptr_fx, 1);
            if( sub(st_fx->ho_hist_ptr_fx,HO_HIST_SIZE) == 0 )
            {
                st_fx->ho_hist_ptr_fx = 0;
                move16();
            }

            /* update the circular buffer of old LSP vectors with the new LSP vector */
            Copy( lsp_new, &(st_fx->ho_lsp_hist_fx[(st_fx->ho_hist_ptr_fx)*M]), M );

            /* update the hangover energy buffer */
            st_fx->ho_ener_hist_fx[st_fx->ho_hist_ptr_fx] = st_fx->Enew_fx;
            move32();
            test();
            IF ( L_sub(core_brate,SID_2k40) == 0 && *sid_bw == 0 )
            {
                /*  enr1 = (float)log10( st->Enew*L_frame + 0.1f ) / (float)log10( 2.0f );*/
                exp = norm_l(st_fx->Enew_fx);
                L_tmp = L_shl(st_fx->Enew_fx,exp);/*Q(exp+6)*/
                L_tmp = Mult_32_16(L_tmp,shl(L_frame,5));/*Q(exp+6+5-15=exp-4)*/
                L_tmp = L_shr(L_tmp,sub(exp,10));/*Q6*/

                exp = norm_l(L_tmp);
                fra = Log2_norm_lc(L_shl(L_tmp,exp));
                exp = sub(sub(30,exp),6);
                L_tmp = L_Comp(exp,fra);
                enr1 = L_shr(L_tmp,10);/* Q6 */

                FOR ( i=0; i<NUM_ENV_CNG; i++ )
                {
                    /* get quantized envelope */
                    /*  env[i] = pow(2.0f,(enr1 - q_env[i])) + 2*st->Enew;*/
                    L_tmp  = L_sub(enr1,q_env[i]);/* Q6 */
                    L_tmp = L_shl(L_tmp, 10);/* 16 */
                    temp_lo_fx = L_Extract_lc(L_tmp, &temp_hi_fx);

                    exp_pow = sub(14, temp_hi_fx);
                    L_tmp = Pow2(14, temp_lo_fx);      /* Qexp_pow */
                    env[i] = L_shl(L_tmp, sub(6, exp_pow));   /* Q6 */
                    L_tmp = L_add(st_fx->Enew_fx,st_fx->Enew_fx);
                    env[i] = L_add(env[i],L_tmp); /* Q6 */    move32();
                }
                Copy32( env, &(st_fx->ho_env_hist_fx[(st_fx->ho_hist_ptr_fx)*NUM_ENV_CNG]), NUM_ENV_CNG );
            }

            st_fx->ho_hist_size_fx = add(st_fx->ho_hist_size_fx,1);
            if( sub(st_fx->ho_hist_size_fx,HO_HIST_SIZE) > 0 )
            {
                st_fx->ho_hist_size_fx = HO_HIST_SIZE;
                move16();
            }
        }
        /* Update the frame length memory */
        st_fx->last_CNG_L_frame_fx = st_fx->L_frame_fx;
        move16();

        if( L_sub(core_brate,SID_1k75) != 0 )
        {
            st_fx->num_ho_fx = m;
            move16();
        }
    }

    /* Update the frame length memory */
    st_fx->last_CNG_L_frame_fx = st_fx->L_frame_fx;

    if( L_sub(core_brate,SID_1k75) != 0 )
    {
        st_fx->num_ho_fx = m;
        move16();
    }

    IF ( st_fx->Opt_AMR_WB_fx )
    {
        E_LPC_f_isp_a_conversion( st_fx->lspCNG_fx, Aq, M );
    }
    ELSE
    {
        E_LPC_f_lsp_a_conversion( st_fx->lspCNG_fx, Aq, M );
    }

    tmp_loop = shr(L_frame,6);
    FOR( i=1; i<tmp_loop; i++ ) /* L_frame/L_SUBFR */
    {
        Copy( Aq, &Aq[i*(M+1)], M+1 );
    }

    return;
}
/*---------------------------------------------------------------------*
 * swb_CNG_dec()
 *
 * Comfort noise generation for SHB signal
 *---------------------------------------------------------------------*/
void swb_CNG_dec_fx(
    Decoder_State_fx *st_fx,               /* i/o: State structure                          */
    const Word16 *synth_fx,            /* i  : ACELP core synthesis at 32kHz            */
    Word16 *shb_synth_fx,        /* o  : high-band CNG synthesis                  */
    const Word16 sid_bw                /* i  : 0-NB/WB, 1-SWB SID                       */
    ,const Word16 Qsyn                  /* i  : Q value of ACELP core synthesis          */
)
{
    test();
    IF ( st_fx->core_brate_fx == FRAME_NO_DATA || L_sub(st_fx->core_brate_fx, SID_2k40) == 0  )
    {
        /* SHB SID decoding and CNG */
        test();
        IF (st_fx->cng_type_fx == LP_CNG && sub(st_fx->extl_fx, SWB_CNG) == 0)
        {
            shb_CNG_decod_fx( st_fx, synth_fx, shb_synth_fx, sid_bw, Qsyn );
        }
        st_fx->last_vad_fx = 0;
        move16();
        st_fx->burst_cnt_fx = 0;
        move16();
    }
    ELSE
    {
        st_fx->last_vad_fx = 1;
        move16();
        st_fx->burst_cnt_fx = add(st_fx->burst_cnt_fx, 1);
        if ( sub(st_fx->burst_cnt_fx, 10) > 0 )
        {
            st_fx->burst_cnt_fx = 0;
            move16();
        }
    }

    return;
}

/*---------------------------------------------------------------------*
 * shb_CNG_decod()
 *
 * Main routine of SHB SID decoding and CNG
 *---------------------------------------------------------------------*/
static void shb_CNG_decod_fx(
    Decoder_State_fx *st_fx,               /* i/o: State structure                          */
    const Word16 *synth_fx,            /* i  : ACELP core synthesis at 32kHz            */
    Word16 *shb_synth_fx,        /* o  : high-band CNG synthesis                  */
    const Word16 sid_bw                /* i  : 0-NB/WB, 1-SWB SID                       */
    ,const Word16 Qsyn                  /* i  : Q value of ACELP core synthesis          */
)
{
    Word16 i;
    Word16 idx_ener_fx;
    Word16 shb_lpcCNG_fx[LPC_SHB_ORDER+1];
    Word16 shb_lspCNG_fx[LPC_SHB_ORDER];
    Word16 excTmp_fx[L_FRAME16k];
    Word16 excSHB_fx[L_FRAME16k];
    Word16 tmp_lsp[LPC_SHB_ORDER];
    Word16 ener_excSHB_fx;
    Word32 wb_ener_fx;
    Word16 wb_ener16_fx;
    Word16 gain_fx;
    Word16 shb_syn16k_fx[L_FRAME16k];
    Word16 tmp;
    Word16 step_fx;
    Word16 interp_fx;
    Word16 ener_fx;
    Word16 exp,exp1;
    Word16 fra;
    Word32 L_tmp;
    Word16 tmp2;
    Word16 allow_cn_step_fx=0;


    IF ( st_fx->bfi_fx == 0 )
    {
        test();
        IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 && sub(sid_bw, 1) == 0 )
        {
            idx_ener_fx = get_next_indice_fx(st_fx, 4);

            if ( idx_ener_fx == 0 )
            {
                idx_ener_fx = -15;
                move16();
            }

            /* de-quantization of SHB CNG parameters */
            L_tmp = L_mult(idx_ener_fx, 27400); /*Q14*/
            st_fx->last_shb_cng_ener_fx = extract_l(L_shr(L_sub(L_tmp, 295924), 6)); /*Q8 */
        }
    }

    /* SHB spectrum estimation */


    interp_fx = s_min(st_fx->shb_dtx_count_fx,32);
    interp_fx = shl(interp_fx, 10); /*Q15*/

    FOR ( i=0; i<LPC_SHB_ORDER; i++ )
    {
        tmp2 = mult(interp_fx, st_fx->lsp_shb_prev_fx[i]);     /*Q14*/
        tmp  = mult(sub(32767, interp_fx), st_fx->lsp_shb_prev_prev_fx[i]); /*Q14*/
        shb_lspCNG_fx[i] = add(tmp2, tmp);
        move16(); /*Q14*/
    }

    if ( sub(st_fx->shb_dtx_count_fx, 1000) < 0 )
    {
        st_fx->shb_dtx_count_fx = add(st_fx->shb_dtx_count_fx, 1);
    }

    E_LPC_lsf_lsp_conversion(shb_lspCNG_fx, tmp_lsp, LPC_SHB_ORDER); /*Q14*/
    E_LPC_f_lsp_a_conversion(tmp_lsp, shb_lpcCNG_fx, LPC_SHB_ORDER);

    Copy_Scale_sig( shb_lpcCNG_fx, shb_lpcCNG_fx, LPC_SHB_ORDER+1, sub(norm_s(shb_lpcCNG_fx[0]),2) );  /* Q12 */



    /* SHB energy estimation */
    wb_ener_fx = L_deposit_l(1);/*Q1 */
    FOR ( i=0; i<L_FRAME32k; i++ )
    {
        wb_ener_fx = L_add(wb_ener_fx, Mpy_32_16_1(L_mult0(synth_fx[i], synth_fx[i]), 51)); /* 2*Qsyn */
    }
    exp = norm_l(wb_ener_fx);
    fra = Log2_norm_lc(L_shl(wb_ener_fx, exp));
    exp = sub(30, add(exp, shl(Qsyn, 1)));
    wb_ener_fx = Mpy_32_16(exp, fra, LG10);
    wb_ener16_fx = round_fx(L_shl(wb_ener_fx, 10)); /*wb_ener_fx in Q8 */
    if ( st_fx->first_CNG_fx == 0 )
    {
        st_fx->wb_cng_ener_fx = wb_ener16_fx;
        move16();/*Q8 */
    }
    if ( sub(abs_s(sub(wb_ener16_fx,st_fx->wb_cng_ener_fx)),3072) > 0 )
    {
        allow_cn_step_fx = 1;
        move16();
    }

    IF ( sub(allow_cn_step_fx, 1) == 0 )
    {
        st_fx->wb_cng_ener_fx = wb_ener16_fx;
        move16(); /*Q8 */
    }
    ELSE
    {
        tmp = sub(wb_ener16_fx, st_fx->wb_cng_ener_fx); /*Q8 */
        tmp = mult_r(tmp, 29491); /*Q8 */
        st_fx->wb_cng_ener_fx = add(st_fx->wb_cng_ener_fx, tmp); /*Q8 */
    }
    test();
    test();
    IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0  && sub(sid_bw, 1) == 0 && st_fx->bfi_fx == 0 )
    {
        st_fx->last_wb_cng_ener_fx = st_fx->wb_cng_ener_fx;
        move16();

        if ( st_fx->first_CNG_fx == 0 )
        {
            st_fx->shb_cng_ener_fx = st_fx->last_shb_cng_ener_fx;
            move16();
        }
    }

    gain_fx = sub(st_fx->wb_cng_ener_fx, st_fx->last_wb_cng_ener_fx); /*8 */
    if (sub(gain_fx, 15) > 0)
    {
        gain_fx = 15;
        move16();
    }
    step_fx = sub(add(gain_fx, st_fx->last_shb_cng_ener_fx), st_fx->shb_cng_ener_fx); /*Q8 */
    test();
    IF ( sub(allow_cn_step_fx,1) == 0 || L_sub(st_fx->last_core_brate_fx,SID_2k40) > 0 )
    {
        st_fx->shb_cng_ener_fx = add(st_fx->shb_cng_ener_fx, step_fx);
    }
    ELSE
    {
        st_fx->shb_cng_ener_fx = add(st_fx->shb_cng_ener_fx, mult(8192, step_fx)); /*Q8 */
    }
    /* generate white noise excitation */
    FOR ( i=0; i<L_FRAME16k; i++ )
    {
        excTmp_fx[i] = Random(&st_fx->swb_cng_seed_fx);
        move16();/*Q15*/ /*normalized*/
    }

    /* synthesis filtering */
    Syn_filt_s( 0, shb_lpcCNG_fx, LPC_SHB_ORDER, excTmp_fx, excSHB_fx, L_FRAME16k, st_fx->state_lpc_syn_fx, 1 );


    /* synthesis signal gain shaping */
    ener_excSHB_fx = 1;
    FOR ( i=0; i<L_FRAME16k; i++ )
    {
        excSHB_fx[i] = shr(excSHB_fx[i], 8);
        L_tmp = Mpy_32_16_1(L_mult0(excSHB_fx[i], excSHB_fx[i]), 102);
        ener_excSHB_fx = add(ener_excSHB_fx, extract_l(L_tmp)); /*Q0 */
    }
    IF ( sub(st_fx->last_vad_fx, 1) == 0 )
    {
        st_fx->trans_cnt_fx = 0;
        move16();
        test();
        if ( sub(st_fx->burst_cnt_fx, 3) > 0 && sub(st_fx->last_core_fx, HQ_CORE) != 0 )
        {
            st_fx->trans_cnt_fx = 5;
            move16();
        }
    }

    ener_fx = st_fx->shb_cng_ener_fx;
    move16();/*Q8 */
    IF ( st_fx->trans_cnt_fx > 0 )
    {
        i = extract_l(L_mult0(st_fx->trans_cnt_fx, 17)); /*Q0 */
        ener_fx = add(st_fx->shb_cng_ener_fx, mult(sin_table256_fx[i], sub(st_fx->last_shb_ener_fx, st_fx->shb_cng_ener_fx))); /*Q8 */
        st_fx->trans_cnt_fx = sub(st_fx->trans_cnt_fx, 1);
    }

    tmp = mult(3277, ener_fx); /*Q8 */
    L_tmp = L_mult(27213, tmp); /*Q22, 27213=3.321928 in Q13  */
    L_tmp = L_shr(L_tmp, 6); /*Q16 */
    L_tmp = L_add(L_tmp, 5<<16);
    IF ( L_tmp <= 0 )
    {
        L_tmp = 1<<3; /*Q5*/
    }
    ELSE
    {
        fra = L_Extract_lc(L_tmp, &exp);
        L_tmp = L_shl(Pow2(exp, fra), 5); /*Q5 */
        L_tmp = L_shr(L_tmp, 5); /*Q5*/
    }
    exp = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, exp); /*Q31*/
    tmp = extract_h(L_tmp); /*Q15*/
    exp = sub(exp, 16);
    exp1 = norm_s(ener_excSHB_fx);
    fra = shl(ener_excSHB_fx, exp1); /*Q15*/

    IF ( sub(fra,tmp) > 0 )
    {
        fra = shr(fra, 1); /*Q15*/
        exp1 = sub(exp1, 1);
    }
    tmp = div_s(fra, tmp); /*Q15*/

    L_tmp = L_deposit_h(tmp); /*Q31 */
    tmp = sub(add(exp,5), exp1);

    L_tmp = Isqrt_lc(L_tmp, &tmp); /*Q31-Qtmp */

    gain_fx = extract_h(L_shl(L_tmp, tmp));

    FOR ( i=0; i<L_FRAME16k; i++ )
    {
        shb_syn16k_fx[i] = extract_l(Mpy_32_16_1(excSHB_fx[i], gain_fx));  /*Q23 */
    }

    test();
    IF(sub(st_fx->last_extl_fx, SWB_TBE) == 0 || sub(st_fx->last_extl_fx, FB_TBE) == 0 )
    {
        /* rescale the Hilbert memories to Q0 */
        FOR(i = 0; i < HILBERT_MEM_SIZE ; i++)
        {
            st_fx->genSHBsynth_Hilbert_Mem_fx[i] = L_shr(st_fx->genSHBsynth_Hilbert_Mem_fx[i], st_fx->prev_Q_bwe_syn2);
            move32();
        }

        FOR(i = 0; i < (2*ALLPASSSECTIONS_STEEP+1); i++ )
        {
            st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx[i] = shr(st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx[i], st_fx->prev_Q_bwe_syn2);
        }

    }
    GenSHBSynth_fx( shb_syn16k_fx, shb_synth_fx, st_fx->genSHBsynth_Hilbert_Mem_fx, st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx, st_fx->L_frame_fx, &(st_fx->syn_dm_phase_fx) );

    IF ( L_sub(st_fx->output_Fs_fx,48000) == 0 )
    {
        interpolate_3_over_2_allpass_fx( shb_synth_fx, L_FRAME32k, shb_synth_fx, st_fx->interpol_3_2_cng_dec_fx, allpass_poles_3_ov_2 );
    }

    ResetSHBbuffer_Dec_fx( st_fx );

    return;
}



