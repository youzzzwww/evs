/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_enc_fx.h"    /* Encoder static table prototypes        */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_mpy.h"

/*---------------------------------------------------------------------*
* Local constants
*---------------------------------------------------------------------*/

#define MAX_DELTA         1
#define MIN_CNT           50      /* Minimum frame number before SID interval adaptation */
#define INT_H             50
#define INT_L             8

/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/
static void shb_CNG_encod_fx(
    Encoder_State_fx *st_fx,             /* i/o: State structure                                 */
    const Word16 update_fx           /* i  : SID update flag                                 */
);
static Word16 shb_DTX_fx(
    Encoder_State_fx *st_fx,             /* i/o: State structure                                 */
    const Word16 *shb_speech_fx,     /* i  : SHB target signal (6-14kHz) at 16kHz            */
    const Word16 *syn_12k8_16k       /* i  : ACELP core synthesis at 12.8kHz or 16kHz        */
);

void CNG_enc_fx(
    Encoder_State_fx *st_fx,/* i/o: State structure                                     */
    const Word16 L_frame,   /* i  : length of the frame                           Q0    */
    Word16 Aq[],            /* o  : LP coefficients                               Q12   */
    const Word16 *speech,   /* i  : pointer to current frame input speech buffer  Q_new */
    Word32 L_enr,           /* i  : residual energy from Levinson-Durbin          Q6    */
    Word16 *lsp_new,        /* i/o: current frame ISPs                            Q15   */
    Word16 *lsf_new,        /* i/o: current frame ISFs                            Qlog2(2.56) */
    Word16 *allow_cn_step,  /* o  : allow CN step                                 Q0    */
    Word16 burst_ho_cnt,    /* i  : hangover frames at end of speech burst        Q0    */
    Word16 Q_new            /* i  : Q value of speech                                   */
    ,Word32 *q_env
    ,Word16 *sid_bw
    ,Word16 *exc_mem2
)
{
    Word16 enr_index;
    Word16 i, j, ptr;
    Word16 res[L_FRAME16k];
    Word16  step_inv=0;
    Word16  hi, lo;
    Word16 maxl=0;
    Word16 num_bits=0;
    Word16 step=0;
    Word16 *pt_res;
    const Word16 *pt_sp;
    Word16 enr;
    Word32 L_tmp, L_ener;
    Word16 k, tmp1;
    Word16 weights;
    Word16 sp_enr;
    Word32 L_tmp1;
    Word16 exp;
    Word16 m = 0;
    Word16 tmp[HO_HIST_SIZE*M];
    Word16 ll, s_ptr;
    Word16 tmpv, maxv, scale, att = 1;
    Word16 lsf_tmp[M];
    Word32 C[M];
    Word32 max[2];
    Word16 max_idx[2];
    Word16 ftmp_fx;
    Word16 lsp_tmp[M];
    Word16 dev;
    Word16 max_dev;
    Word16 dist;
    Word16 max_idx1[2]= {0,0};
    Word16 fft_io[L_FRAME16k];
    Word16 *ptR,*ptI;
    Word32 enr1;
    Word32 env[NUM_ENV_CNG];
    Word32 min1;
    Word16 min1_idx;
    Word32 d;
    Word16 res1[L_FRAME16k];
    Word32 tmp_env[HO_HIST_SIZE*NUM_ENV_CNG];
    Word16 fra;
    Word16 temp_lo_fx, temp_hi_fx;
    Word16 exp_pow;
    Word16 force_cn_step=0;
    Word16 tmp_loop;

    /* Temp variables for floating point functions */

    /*sp_enr = (float) log10( sum2_f( speech, L_frame )/L_frame + 0.1f )/ (float)log10(2.0f);*//*9.1 */
    pt_sp = speech;
    L_ener = L_deposit_l(1);
    /*  L_ener = L_add(L_shr(sum2_f_fx( speech, L_frame ), 8) , L_ener);*/
    IF( sub(L_frame, L_FRAME) == 0)
    {
        FOR (j=0; j<128; j++)
        {
            L_tmp = L_mult0(*pt_sp, *pt_sp);
            pt_sp++;
            L_tmp = L_mac0(L_tmp, *pt_sp, *pt_sp);
            pt_sp++;
            L_ener = L_add(L_ener, L_shr(L_tmp, 7)); /* 2*Q_new + 1, divide by L_frame done here */
        }
    }
    ELSE /* L_FRAME16k */
    {
        FOR (i=0; i<2; i++)
        {
            FOR (j=0; j<80; j++)
            {
                L_tmp = L_mult0(*pt_sp, *pt_sp);
                pt_sp++;
                L_tmp = L_mac0(L_tmp, *pt_sp, *pt_sp);
                pt_sp++;
                L_ener = L_add(L_ener, L_shr(Mult_32_16(L_tmp,26214 /* 256/320, Q15 */), 7)); /* 2*Q_new + 1, divide by L_frame done here */
            }
        }
    }

    hi = norm_l(L_ener);
    lo = Log2_norm_lc(L_shl(L_ener, hi));
    hi = sub(29, hi);   /* log2 exp in Q2*Q_new */
    hi = sub(hi, shl(Q_new, 1)); /* Q0 */
    L_tmp = L_Comp(hi, lo); /* Q16 */
    sp_enr = round_fx(L_shl(L_tmp, 8)); /* Q8 (16+8-16) */

    if (sp_enr < 0)
    {
        sp_enr = 0;
        move16();
    }
    test();
    IF ( st_fx->first_CNG_fx == 0 || st_fx->old_enr_index_fx < 0 )
    {
        st_fx->lp_sp_enr_fx = sp_enr;
        move16();  /* Q8 */
    }
    ELSE
    {
        test();
        test();
        test();
        test();
        IF ( L_sub(st_fx->last_core_brate_fx, SID_2k40) > 0 && burst_ho_cnt > 0 && sub(st_fx->lp_sp_enr_fx, 1536) < 0 &&
        sub(sub(sp_enr, st_fx->lp_sp_enr_fx), 1024) > 0 && sub(sp_enr, 1536) > 0 )
        {
            st_fx->lp_sp_enr_fx = sp_enr;
            move16();
            force_cn_step = 1;
            move16();
        }
        ELSE
        {
            st_fx->lp_sp_enr_fx = round_fx(L_mac(L_mult(29491 /* 0.9, Q15 */,st_fx->lp_sp_enr_fx), 3277 /* 0.1, Q15 */,sp_enr)); /* Q8 (8+15+1-16) */
        }
    }
    /* update the pointer to circular buffer of old LSP vectors */
    st_fx->cng_hist_ptr_fx = add(st_fx->cng_hist_ptr_fx,1);
    if(sub(st_fx->cng_hist_ptr_fx, DTX_HIST_SIZE) == 0)
    {
        st_fx->cng_hist_ptr_fx = 0;
        move16();
    }

    /* update the circular buffer of old LSP vectors with the new LSP vector */
    Copy( lsp_new, &(st_fx->cng_lsp_hist_fx[(st_fx->cng_hist_ptr_fx)*M]), M );

    /*-----------------------------------------------------------------*
    * Find CNG spectral envelope
    * Find LSP median
    *-----------------------------------------------------------------*/
    test();
    test();
    IF( (L_sub(st_fx->core_brate_fx, SID_2k40) == 0 || L_sub(st_fx->core_brate_fx, SID_1k75) == 0) && sub(st_fx->cng_cnt_fx, sub(st_fx->cng_hist_size_fx,1)) >= 0 )
    {
        set32_fx( max, 0, 2 );
        set16_fx( max_idx, 0, 2 );

        FOR( i=0; i<st_fx->cng_hist_size_fx; i++ )
        {
            IF ( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
            {
                lsp2lsf_fx( &st_fx->cng_lsp_hist_fx[i*M], lsf_tmp, M, INT_FS_FX );
                ftmp_fx = 964;
                move16();/*QX2.56  */
                tmpv = sub(16384,add(lsf_tmp[M-1],ftmp_fx)); /*QX2.56 */
                L_tmp = L_mult0(tmpv,tmpv); /*QX6.5536 */
            }
            ELSE
            {
                lsp2lsf_fx( &st_fx->cng_lsp_hist_fx[i*M], lsf_tmp, M, INT_FS_16k );
                ftmp_fx = 1205;
                move16();/*QX2.56 */
                tmpv = sub(20480,add(lsf_tmp[M-1],ftmp_fx)); /*QX2.56 */
                L_tmp = L_mult0(tmpv,tmpv);  /*QX6.5536 */
            }

            tmpv = sub(lsf_tmp[0],ftmp_fx); /*QX2.56 */
            L_tmp = L_mac0(L_tmp,tmpv,tmpv);  /*QX6.5536       */
            FOR ( j=0; j<M-1; j++ )
            {
                tmpv = sub(sub(lsf_tmp[j+1],lsf_tmp[j]),ftmp_fx); /*QX2.56 */
                L_tmp = L_mac0(L_tmp,tmpv,tmpv);  /*QX6.5536 */
            }

            C[i] = Mpy_32_16_1(L_tmp,1928);
            move32();/*QX6.5536 */

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

        FOR ( i=0; i<M; i++ )
        {
            L_tmp = 0;
            FOR ( j=0; j<st_fx->cng_hist_size_fx; j++ )
            {
                L_tmp = L_add(L_tmp,L_deposit_l(st_fx->cng_lsp_hist_fx[j*M+i])); /*Q15 */
            }

            L_tmp = L_sub(L_tmp,L_add(L_deposit_l(st_fx->cng_lsp_hist_fx[max_idx[0]*M+i]),L_deposit_l(st_fx->cng_lsp_hist_fx[max_idx[1]*M+i]))); /*Q15 */
            tmpv= div_s(1,sub(st_fx->cng_hist_size_fx,2)); /*Q15 */
            L_tmp = Mpy_32_16_1(L_tmp,tmpv); /*Q15 */
            lsp_new[i] = extract_l(L_tmp); /*Q15 */
        }
        max_idx1[0] = max_idx[0];
        move16();
        max_idx1[1] = max_idx[1];
        move16();
    }

    /*-----------------------------------------------------------------*
    * Quantize CNG spectral envelope (only in SID frame)
    * Quantize the LSF vector
    *-----------------------------------------------------------------*/
    *allow_cn_step = 0;
    move16();
    test();
    test();
    test();
    test();
    test();
    test();
    IF( ((st_fx->cng_cnt_fx == 0) &&
         sub(st_fx->lp_sp_enr_fx, 1536) > 0 &&
         (sub(add(st_fx->lp_sp_enr_fx,1024 /* 4.0, Q8 */), sp_enr) < 0) &&
         (st_fx->first_CNG_fx !=0 ) &&
         (st_fx->old_enr_index_fx>=0) &&
         (L_sub(st_fx->last_core_brate_fx, SID_2k40) > 0)) ||
        sub(force_cn_step, 1) == 0 )
    {
        *allow_cn_step = 1;
        move16();
    }
    test();
    IF( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 || L_sub(st_fx->core_brate_fx, SID_1k75) == 0 )
    {
        /* LSF quantization */
        IF ( st_fx->Opt_AMR_WB_fx != 0 )
        {
            isf_enc_amr_wb_fx( st_fx, lsf_new, lsp_new, 0, 0, 0 );
        }
        ELSE
        {
            lsf_enc_fx( st_fx, L_frame, INACTIVE, lsf_new, lsp_new, 0, 0, 0, 100, Q_new );
        }
        /* Reset CNG history if CNG frame length is changed */
        test();
        test();
        if ( sub(st_fx->bwidth_fx,WB) == 0 && st_fx->first_CNG_fx && sub(st_fx->L_frame_fx,st_fx->last_CNG_L_frame_fx) != 0 )
        {
            st_fx->ho_hist_size_fx = 0;
            move16();
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

    /*---------------------------------------------------------------------*
    * CNG spectral envelope update
    * Find A(z) coefficients
    *---------------------------------------------------------------------*/

    IF( L_sub(st_fx->last_core_brate_fx, SID_2k40) <= 0 )
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
                st_fx->lspCNG_fx[i] = mac_r(L_mult(CNG_ISF_FACT_FX,st_fx->lspCNG_fx[i]),32768-CNG_ISF_FACT_FX,lsp_new[i]);
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
        st_fx->act_cnt_fx = 0;
        move16();
        s_ptr = add(sub(st_fx->ho_circ_ptr_fx,burst_ho_cnt),1);

        if( s_ptr < 0 )
        {
            s_ptr = add(s_ptr, st_fx->ho_circ_size_fx);
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
            IF( sub(L_frame,L_FRAME ) == 0 && sub(st_fx->ho_16k_lsp_fx[s_ptr],1) == 0 )
            {
                /* Conversion from 16k LPSs to 12k8 */
                lsp_convert_poly_fx( &(st_fx->ho_lsp_circ_fx[s_ptr*M]), L_frame, 0 );
            }
            ELSE IF ( sub(L_frame,L_FRAME16k) == 0 && st_fx->ho_16k_lsp_fx[s_ptr] == 0 )
            {
                /* 16k LSPs already converted and stored, just copy to the other buffer */
                Copy(&(st_fx->ho_lsp_circ2_fx[s_ptr*M]), &(st_fx->ho_lsp_circ_fx[s_ptr*M]), M );
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

        IF ( burst_ho_cnt > 0)
        {
            /**allow_cn_step |= ( st_fx->ho_ener_hist_fx[st_fx->ho_hist_ptr_fx] > 4 * st_fx->lp_ener_fx ); */
            L_tmp1 = L_shr(st_fx->ho_ener_hist_fx[st_fx->ho_hist_ptr_fx],2);
            L_tmp1 = L_sub(L_tmp1,st_fx->lp_ener_fx);

            if(L_tmp1>0)
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
                    /*enr += W_DTX_HO[k] * st_fx->ho_ener_hist[ptr]; */
                    L_tmp1 = Mult_32_16(st_fx->ho_ener_hist_fx[ptr],W_DTX_HO_FX[k]) ; /* Q6+15-15->Q6 */
                    L_enr = L_add(L_enr,L_tmp1); /* Q6 */

                    /*weights += W_DTX_HO[k]; */
                    weights = add( weights, W_DTX_HO_FX[k]); /* Q15 */

                    Copy( &st_fx->ho_lsp_hist_fx[ptr*M], &tmp[m*M], M );
                    Copy32( &st_fx->ho_env_hist_fx[ptr*NUM_ENV_CNG], &tmp_env[m*NUM_ENV_CNG], NUM_ENV_CNG );
                    m = add(m,1);
                }
            }

            /*enr /= weights; */
            exp = norm_s(weights);
            tmp1 = div_s(shl(1,sub(14,exp)),weights); /* Q(15+14-exp-15) */
            L_tmp1 = Mult_32_16(L_enr,tmp1); /* Q(14-exp+6-15)->Q(5-exp) */
            L_enr = L_shl(L_tmp1,add(exp,1)); /* Q6 */

            st_fx->lp_ener_fx = L_enr;
            move32();/* Q6 */

            set32_fx( max, 0, 2 );
            set16_fx( max_idx, 0, 2 );

            FOR( i=0; i<m; i++ )
            {
                IF ( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
                {
                    lsp2lsf_fx( &tmp[i*M], lsf_tmp, M, INT_FS_FX );
                    ftmp_fx = 964;
                    move16();/*QX2.56  */
                    tmpv = sub(16384,add(lsf_tmp[M-1],ftmp_fx)); /*QX2.56 */
                    L_tmp = L_mult0(tmpv,tmpv); /*QX6.5536 */
                }
                ELSE
                {
                    lsp2lsf_fx( &tmp[i*M], lsf_tmp, M, INT_FS_16k );
                    ftmp_fx = 1205;
                    move16();/*QX2.56 */
                    tmpv = sub(20480,add(lsf_tmp[M-1],ftmp_fx)); /*QX2.56 */
                    L_tmp = L_mult0(tmpv,tmpv); /*QX6.5536 */
                }

                tmpv = sub(lsf_tmp[0],ftmp_fx); /*QX2.56 */
                L_tmp = L_mac0(L_tmp,tmpv,tmpv); /*QX6.5536       */
                FOR ( j=0; j<M-1; j++ )
                {
                    tmpv = sub(sub(lsf_tmp[j+1],lsf_tmp[j]),ftmp_fx); /*QX2.56 */
                    L_tmp = L_mac0(L_tmp,tmpv,tmpv); /*QX6.5536 */
                }

                C[i] = Mpy_32_16_1(L_tmp,1928);
                move32();/*QX6.5536 */

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
                    L_tmp = L_deposit_l(0);
                    FOR ( j=0; j<m; j++ )
                    {
                        L_tmp = L_add(L_tmp,L_deposit_l(tmp[j*M+i]));
                    }

                    L_tmp = L_sub(L_tmp,L_deposit_l(tmp[max_idx[0]*M+i]));
                    tmpv= div_s(1,sub(m,1)); /*Q15 */
                    L_tmp = Mpy_32_16_1(L_tmp,tmpv); /*Q15 */
                    lsp_tmp[i] = extract_l(L_tmp); /*Q15 */
                }
            }
            ELSE
            {
                FOR ( i=0; i<M; i++ )
                {
                    L_tmp = L_deposit_l(0);
                    FOR ( j=0; j<m; j++ )
                    {
                        L_tmp = L_add(L_tmp,L_deposit_l(tmp[j*M+i]));
                    }

                    L_tmp = L_sub(L_tmp,L_add(L_deposit_l(tmp[max_idx[0]*M+i]),L_deposit_l(tmp[max_idx[1]*M+i]))); /*Q15 */
                    tmpv= div_s(1,sub(m,2)); /*Q15 */
                    L_tmp = Mpy_32_16_1(L_tmp,tmpv); /*Q15 */
                    lsp_tmp[i] = extract_l(L_tmp); /*Q15 */
                }
            }

            dist = 0; /*Q15 */
            max_dev = 0; /*Q15 */
            FOR ( i=0; i<M; i++ )
            {
                dev = abs_s(sub(lsp_tmp[i],lsp_new[i])); /*Q15 */
                dist = add(dist,dev); /*Q15 */
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
                    move16(); /*Q15 */
                }
            }
            ELSE
            {
                FOR( i=0; i<M; i++ )
                {
                    /* AR low-pass filter  */
                    st_fx->lspCNG_fx[i] = add(mult_r(26214,lsp_tmp[i]),mult_r(6554,lsp_new[i]));
                    move16();
                }
            }
            FOR ( i=0; i<NUM_ENV_CNG; i++ )
            {
                L_tmp = L_deposit_l(0);
                FOR ( j=0; j<m; j++ )
                {
                    /* env[i] += tmp_env[j*NUM_ENV_CNG+i]; */
                    L_tmp = L_add(L_tmp,tmp_env[j*NUM_ENV_CNG+i]);
                }
                /*    env[i] /= (float)m;  */
                /*    env[i] = env[i] - 2*st_fx->lp_ener_fx; */
                IF(sub(m,1) == 0)
                {
                    L_tmp = L_sub(L_tmp,L_add(st_fx->lp_ener_fx,st_fx->lp_ener_fx));
                }
                ELSE
                {
                    tmp1 = div_s(1,m);
                    L_tmp = Mult_32_16(L_tmp,tmp1);
                    L_tmp = L_sub(L_tmp,L_add(st_fx->lp_ener_fx,st_fx->lp_ener_fx));
                }

                env[i] = L_tmp;
                move32();
            }

            Copy32(env, st_fx->lp_env_fx, NUM_ENV_CNG);
        }
        ELSE
        {
            Copy( lsp_new, st_fx->lspCNG_fx, M );  /* use newly analyzed ISFs */
        }
    }
    IF ( st_fx->Opt_AMR_WB_fx != 0 )
    {
        E_LPC_f_isp_a_conversion( st_fx->lspCNG_fx, Aq, M );
    }
    ELSE
    {
        E_LPC_f_lsp_a_conversion( st_fx->lspCNG_fx, Aq, M );     /* Find A(z) (not interpolated) */
    }

    tmp_loop = shr(L_frame,6);
    FOR( i=1; i<tmp_loop; i++ )
    {
        Copy( Aq, &Aq[i*(M+1)], M+1 );
    }
    /*-----------------------------------------------------------------*
    * Find residual signal
    * Calculate residual signal energy per sample
    *-----------------------------------------------------------------*/

    /* calculate the residual signal */
    Residu3_fx(Aq, speech, res, L_frame, 0);
    Copy(res, res1, L_frame);
    IF( sub(st_fx->bwidth_fx,NB) != 0 )
    {
        test();
        IF( sub(st_fx->bwidth_fx,WB) == 0 && st_fx->CNG_mode_fx >= 0 )
        {
            ftmp_fx = HO_ATT_FX[st_fx->CNG_mode_fx];
        }
        ELSE
        {
            ftmp_fx = 19661;
            move16();
        }

        att = mult(ftmp_fx,5461);/* Q15 */
        L_tmp = L_mult(att,8);/* Q16 */
        tmp1 = extract_l(L_shr(L_tmp,2));/* Q14 */
        tmp1 = add(16384,tmp1);
        att = div_s(16374,tmp1); /* Q15 */

        IF ( sub(att,ftmp_fx) < 0 )
        {
            att = ftmp_fx;
            move16();
        }

        FOR( i = 0; i < st_fx->L_frame_fx; i++ )
        {
            /*res1[i] *= att;*/
            res1[i] = mult(res1[i],att);
            move16();/* Q_new */
        }
        att = shr(att,7);/* Q8 */
    }

    /* calculate the spectrum of residual signal */
    Copy(res1, fft_io, st_fx->L_frame_fx);

    IF ( sub(st_fx->L_frame_fx,L_FRAME16k) == 0 )
    {
        modify_Fs_fx( fft_io, L_FRAME16k, 16000, fft_io, 12800, exc_mem2,0);
    }

    fft_rel_fx(fft_io, L_FFT, LOG2_L_FFT);
    ptR = &fft_io[1];
    ptI = &fft_io[L_FFT-1];
    FOR ( i=0; i<NUM_ENV_CNG; i++ )
    {
        /* env[i] = 2.0f*(*ptR * *ptR + *ptI * *ptI)/L_FFT; */
        L_tmp = L_mult(*ptR,*ptR);/* 2*Q_new+1 */
        L_tmp = L_add(L_tmp,L_mult(*ptI,*ptI));/* 2*Q_new+1 */
        L_tmp = L_add(L_tmp,L_tmp);/* 2*Q_new+1 */
        L_tmp = Mult_32_16(L_tmp,128);/* 2*Q_new+1 */
        tmp1 = add(add(Q_new,Q_new),1);
        env[i] = L_shr(L_tmp,sub(tmp1,6));
        move32(); /* Q6 */
        ptR++;
        ptI--;
    }

    Copy32( env, &(st_fx->cng_res_env_fx[(st_fx->cng_hist_ptr_fx)*NUM_ENV_CNG]), NUM_ENV_CNG );

    /* calculate the residual signal energy */
    /*enr = dotp( res, res, L_frame ) / L_frame; */
    maxv = 0;
    move16();
    FOR(i = 0; i < st_fx->L_frame_fx; i++)
    {
        maxv = s_max(maxv, abs_s(res[i]));
    }
    scale = norm_s(maxv);
    pt_res = res;
    L_ener = L_deposit_l(1);
    IF( sub(L_frame, L_FRAME) == 0)
    {
        FOR (j=0; j<128; j++)
        {
            tmpv = shl(*pt_res,scale);
            L_tmp = L_mult0(tmpv, tmpv);
            pt_res++;
            tmpv = shl(*pt_res,scale);
            L_tmp = L_mac0(L_tmp, tmpv, tmpv); /* 2*(Q_new+scale) */
            pt_res++;
            L_ener = L_add(L_ener, L_shr(L_tmp, 7)); /* 2*(Q_new+scale)+1, divide by L_frame done here */
        }
    }
    ELSE /* L_FRAME16k */
    {
        FOR (j=0; j<160; j++)
        {
            tmpv = shl(*pt_res,scale);
            L_tmp = L_mult0(tmpv, tmpv);
            pt_res++;
            tmpv = shl(*pt_res,scale);
            L_tmp = L_mac0(L_tmp, tmpv, tmpv); /* 2*(Q_new+scale) */
            pt_res++;
            L_ener = L_add(L_ener, L_shr(Mult_32_16(L_tmp,26214 /* 256/320, Q15 */), 7)); /* 2*(Q_new+scale)+15+1-16+1, divide by L_frame done here */
        }
    }
    /* convert log2 of residual signal energy */
    /*enr = (float)log10( enr + 0.1f ) / (float)log10( 2.0f ); */
    hi = norm_l(L_ener);
    lo = Log2_norm_lc(L_shl(L_ener, hi));
    hi = sub(29, hi);   /* log2 exp in Q2*(Q_new+scale) */
    hi = sub(hi, shl(add(Q_new,scale), 1)); /* Q0 */
    L_tmp = L_Comp(hi, lo); /* Q16 */
    enr = round_fx(L_shl(L_tmp, 8)); /* Q8 (16+8-16) */

    /* update the circular buffer of old energies */
    st_fx->cng_ener_hist_fx[st_fx->cng_hist_ptr_fx] = enr;
    move16();  /* Q8 */

    /*-----------------------------------------------------------------*
    * Quantize residual signal energy (only in SID frame)
    *-----------------------------------------------------------------*/
    test();
    IF( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 || L_sub(st_fx->core_brate_fx, SID_1k75) == 0 )
    {
        IF( sub(st_fx->cng_cnt_fx,sub(st_fx->cng_hist_size_fx,1)) >= 0 )
        {
            /* average the envelope except outliers */
            FOR ( i=0; i<NUM_ENV_CNG; i++ )
            {
                L_tmp1 = L_add(env[i], 0);
                FOR ( j=0; j<st_fx->cng_hist_size_fx; j++ )
                {
                    L_tmp1 = L_add(L_tmp1,st_fx->cng_res_env_fx[j*NUM_ENV_CNG+i]);
                }
                L_tmp = L_add(st_fx->cng_res_env_fx[max_idx1[0]*NUM_ENV_CNG+i],st_fx->cng_res_env_fx[max_idx1[1]*NUM_ENV_CNG+i]);
                L_tmp1 = L_sub(L_tmp1,L_tmp);

                /*      env[i] /= (float)(st_fx->cng_hist_size_fx - 2);   */
                tmp1 = sub(st_fx->cng_hist_size_fx,2);
                IF(sub(tmp1,1) > 0)
                {
                    tmp1 = div_s(1,tmp1);
                    L_tmp1 = Mult_32_16(L_tmp1,tmp1);
                }

                env[i] = L_tmp1;
                move32();
            }
            /* compute average excitation energy */
            L_tmp = L_deposit_l(0);
            ptr = st_fx->cng_hist_ptr_fx;
            move16();

            FOR (k=0; k<st_fx->cng_hist_size_fx; k++)
            {
                /* enr += W_HIST[k]*cng_ener_hist[ptr] */
                L_tmp = L_mac0(L_tmp, W_HIST_FX[k], st_fx->cng_ener_hist_fx[ptr]); /* Q24 (8+16) */
                ptr = sub(ptr, 1);
                if (ptr < 0)    /* check for circular pointer */
                {
                    ptr = DTX_HIST_SIZE - 1;
                    move16();
                }
            }
            /*-----------------------------------------------------------
            * here we want to divide L_tmp by the sum
            * of all the coefs used W_HIST[0..cng_hist_size-1]
            * The table W_HIST_S already contains the inverted sum.
            * That is
            *  W_HIST_S[k]                 1
            * ------------- = ---------------------------
            *     4096        W_HIST[0] + ... + W_HIST[k]
            *
            * 1 / Sum(W_HIST[0..k]) is always > 1 since the sum
            * of the coefs 0..k is always < 1 but > 0
            * enr is in Q8 since the history buffer constains Q8 energies
            *-----------------------------------------------------------*/
            /*L_tmp = Mpy_32_16_1(L_tmp, W_HIST_S_FX[k-1]);   */ /* normalize average value */
            L_tmp = Mult_32_16(L_tmp, W_HIST_S_FX[k-1]); /* Q21 (24+12+1-16) */
            L_tmp = L_shl(L_tmp, 3); /* Q24 */
            enr = round_fx(L_tmp); /* Q8 */
        }
        /* decrease the energy in case of WB input */
        IF( sub(st_fx->bwidth_fx, NB) != 0 )
        {
            IF( sub(st_fx->bwidth_fx,WB) == 0 )
            {
                IF( st_fx->CNG_mode_fx >= 0 )
                {
                    /* Bitrate adapted attenuation */
                    att = ENR_ATT_fx[st_fx->CNG_mode_fx];
                    move16();
                }
                ELSE
                {
                    /* Use least attenuation for higher bitrates */
                    att = ENR_ATT_fx[4];
                    move16();
                }
            }
            ELSE
            {
                att = 384;
                move16();/*Q8*/
            }
            enr = sub(enr, att );
        }
        /* intialize the energy quantization parameters */
        IF( st_fx->Opt_AMR_WB_fx == 0 )
        {
            step = STEP_SID_FX;
            move16();
            step_inv = ISTEP_SID_FX;
            move16();
            maxl = 127;
            move16();
            num_bits = 7;
            move16();
        }
        ELSE
        {
            step = STEP_AMR_WB_SID_FX;
            move16();
            step_inv = ISTEP_AMR_WB_SID_FX;
            move16();
            maxl = 63;
            move16();
            num_bits = 6;
            move16();
        }

        /* calculate the energy quantization index */
        enr_index = add(enr, 512 /* Q8(2.0) */);   /* enr + 2.0 */
        enr_index = extract_l(L_shr(L_mult0(enr_index, step), 12+8));   /* Q0 (8+12-(8+12)) */

        /* limit the energy quantization index */
        enr_index = s_min(enr_index, maxl);
        enr_index = s_max(enr_index, 0);

        /* allow only slow energy increase */
        test();
        IF( st_fx->old_enr_index_fx >= 0 && sub(enr_index, add(st_fx->old_enr_index_fx, MAX_DELTA)) > 0 )
        {
            IF( *allow_cn_step != 0 )
            {
                tmp1 = mult(27853 /* Q15(0.85) */,sub(enr_index,st_fx->old_enr_index_fx));
                enr_index = add(st_fx->old_enr_index_fx,tmp1);
            }
            ELSE
            {
                enr_index = add(st_fx->old_enr_index_fx, MAX_DELTA);
            }
        }
        st_fx->old_enr_index_fx = enr_index;
        move16();

        push_indice_fx( st_fx, IND_ENERGY, enr_index, num_bits );
        if ( enr_index == 0 )
        {
            enr_index = -5;
            move16();
        }
        /* Find quantized energy */
        /* *Enew = (float)enr_index / step - 2.0 */
        /* *Enew = (float)pow(2.0, *Enew) */
        L_tmp = L_mult(enr_index, step_inv);  /* Q16(0+15+1) */
        /* substract by 2 not done to leave Energy in Q2 */
        lo = L_Extract_lc(L_tmp, &hi);
        st_fx->Enew_fx = Pow2(add(hi, 4), lo); /* Q6 */

        /* enr1 = (float)log10( st->Enew*L_frame + 0.1f ) / (float)log10( 2.0f );*/
        exp = norm_l(st_fx->Enew_fx);
        L_tmp = L_shl(st_fx->Enew_fx,exp); /*Q(exp+6) */
        L_tmp = Mult_32_16(L_tmp,shl(L_frame,5));/* Q(exp+6+5-15=exp-4) */
        L_tmp = L_shr(L_tmp,sub(exp,10)); /* Q6 */

        exp = norm_l(L_tmp);
        fra = Log2_norm_lc(L_shl(L_tmp,exp));
        exp = sub(sub(30,exp),6);
        L_tmp = L_Comp(exp,fra);
        enr1 = L_shr(L_tmp,10);/* Q6 */

        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            /*  env[i] -= 2 * st->Enew;*/
            L_tmp1 = L_add(env[i], 0);
            L_tmp = L_add(st_fx->Enew_fx,st_fx->Enew_fx);
            L_tmp1 = L_sub(L_tmp1,st_fx->Enew_fx); /*Q6*/

            IF ( L_tmp1 < 0 )
            {
                L_tmp1 = L_deposit_l(6); /* (0.1)Q6 */
            }
            /*    env[i] = (float)log10( env[i] + 0.1f ) / (float)log10( 2.0f ); */
            exp = norm_l(L_tmp1);
            fra = Log2_norm_lc(L_shl(L_tmp1,exp));
            exp = sub(sub(30,exp),6);
            L_tmp = L_Comp(exp,fra);
            L_tmp1 = L_shr(L_tmp,10);   /* Q6 */

            L_tmp = L_shr(L_deposit_l(att),2);/* Q6 */
            L_tmp1 = L_sub(L_tmp1,L_tmp);

            IF ( L_tmp1 < 0 )
            {
                L_tmp1 = L_deposit_l(0);
            }

            L_tmp1 = L_sub(enr1,L_tmp1);

            env[i] = L_tmp1;
            move32();
        }

        /* codebook search */
        min1 = L_add(1310588928, 0); /* Q17 */
        min1_idx = 0;
        move16();

        FOR ( i=0; i<64; i++ )
        {
            d = L_deposit_l(0);
            FOR ( j=0; j<NUM_ENV_CNG; j++ )
            {
                /* d += (env[j] - CNG_details_codebook_fx[i][j]) * (env[j] - CNG_details_codebook_fx[i][j]);*/
                L_tmp = L_sub(env[j],CNG_details_codebook_fx[i][j]);/* Q6 */
                exp = norm_l(L_tmp);
                L_tmp = L_shl(L_tmp,exp);/*Q(exp+6)*/
                tmp1 = extract_h(L_tmp);/*Q(exp+6-16)=exp-10*/
                L_tmp = L_mult(tmp1,tmp1);/*Q(2*exp - 19)*/
                L_tmp = L_shr(L_tmp,sub(add(exp,exp),36));/* Q17 */
                d = L_add(d,L_tmp);

            }

            IF ( L_sub(d,min1) < 0 )
            {
                min1 = L_add(d, 0);
                min1_idx = i;
                move16();
            }
        }

        IF ( L_sub(st_fx->core_brate_fx,SID_2k40) == 0 )
        {
            push_indice_fx( st_fx, IND_CNG_ENV1, min1_idx, 6 );
        }

        /* get quantized res_env_details */
        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            q_env[i] = CNG_details_codebook_fx[min1_idx][i];
            move32();
        }

        /* Update hangover memory during CNG */
        test();
        IF ( *allow_cn_step == 0 && L_sub(Mult_32_16(st_fx->Enew_fx,21845 /*1/1.5f, Q15*/), st_fx->lp_ener_fx) < 0 )
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
            FOR ( i=0; i<NUM_ENV_CNG; i++ )
            {
                /* get quantized envelope */
                /*  env[i] = pow(2.0f,(enr1 - q_env[i])) + 2*st->Enew;*/
                L_tmp  = L_sub(enr1,q_env[i]);/* Q6 */
                L_tmp = L_shl(L_tmp, 10);/* 16 */
                temp_lo_fx = L_Extract_lc(L_tmp, &temp_hi_fx);

                exp_pow = sub(14, temp_hi_fx);
                L_tmp = Pow2(14, temp_lo_fx);        /* Qexp_pow */
                env[i] = L_shl(L_tmp, sub(6, exp_pow));
                move32(); /* Q6 */
                L_tmp = L_add(st_fx->Enew_fx,st_fx->Enew_fx);
                env[i] = L_add(env[i],L_tmp);
                move32();/* Q6 */
            }

            Copy32( env, &(st_fx->ho_env_hist_fx[(st_fx->ho_hist_ptr_fx)*NUM_ENV_CNG]), NUM_ENV_CNG );

            st_fx->ho_hist_size_fx = add(st_fx->ho_hist_size_fx,1);
            if( sub(st_fx->ho_hist_size_fx,HO_HIST_SIZE) > 0 )
            {
                st_fx->ho_hist_size_fx = HO_HIST_SIZE;
                move16();
            }
        }
    }
    /* dithering bit for AMR-WB IO mode is always set to 0 */
    IF( L_sub(st_fx->core_brate_fx, SID_1k75) == 0 )
    {
        push_indice_fx( st_fx, IND_DITHERING, 0, 1 );
    }
    IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
    {
        IF(sub(st_fx->L_frame_fx, L_FRAME16k) == 0)
        {
            push_indice_fx( st_fx, IND_ACELP_16KHZ, 1, 1 );
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_ACELP_16KHZ, 0, 1 );
        }
    }

    IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
    {
        push_indice_fx( st_fx, IND_CNG_HO, s_min(st_fx->burst_ho_cnt_fx, 7 ), 3 );


        st_fx->num_ho_fx = m;
        move16();
        push_indice_fx( st_fx, IND_SID_TYPE, 0, 1 );

        IF ( L_sub(st_fx->input_Fs_fx, 32000) < 0 )
        {
            push_indice_fx( st_fx, IND_SID_BW, 0, 1 );
            *sid_bw = 0;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
    * Updates
    *-----------------------------------------------------------------*/
    /* update the SID frames counter */
    test();
    IF( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 || L_sub(st_fx->core_brate_fx, SID_1k75) == 0 )
    {
        st_fx->cng_cnt_fx = 0;
        move16();
        st_fx->cng_hist_ptr_fx = -1;
        move16();
        /* update frame length memory */
        st_fx->last_CNG_L_frame_fx = st_fx->L_frame_fx;
        move16();
    }
    ELSE
    {
        st_fx->cng_cnt_fx = add(st_fx->cng_cnt_fx,1);
    }

    return;
}
/*---------------------------------------------------------------------*
 * swb_CNG_enc()
 *
 * SWB DTX/CNG encoding
 *---------------------------------------------------------------------*/
void swb_CNG_enc_fx(
    Encoder_State_fx *st_fx,             /* i/o: State structure                                                   */
    const Word16 *shb_speech_fx,     /* i  : SHB target signal (6-14kHz) at 16kHz       (Q0)                   */
    const Word16 *syn_12k8_16k_fx    /* i  : ACELP core synthesis at 12.8kHz or 16kHz   (st_fx->Q_syn = 0)     */
)
{
    Word16 shb_SID_updt_fx=0;

    test();
    IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 || st_fx->core_brate_fx == FRAME_NO_DATA  )
    {
        IF (sub(st_fx->cng_type_fx,LP_CNG) == 0)
        {
            /* decide if SHB SID encoding or not */
            shb_SID_updt_fx = shb_DTX_fx( st_fx, shb_speech_fx, syn_12k8_16k_fx );

            /* SHB CNG encoding */
            shb_CNG_encod_fx( st_fx, shb_SID_updt_fx );
        }
        st_fx->last_vad_fx = 0;
        move16();
    }
    ELSE
    {
        st_fx->last_vad_fx = 1;
        move16();
    }

    return;
}

/*---------------------------------------------------------------------*
 * shb_CNG_encod()
 *
 * SID parameters encoding for SHB signal
 *---------------------------------------------------------------------*/
static void shb_CNG_encod_fx(
    Encoder_State_fx *st_fx,             /* i/o: State structure                                 */
    const Word16 update_fx           /* i  : SID update flag                                 */
)
{
    Word16 idx_ener_fx;

    idx_ener_fx = 0;
    move16();
    IF ( sub(update_fx, 1) == 0 )
    {
        /* SHB energy quantization */
        idx_ener_fx = shr(add(mult(st_fx->mov_shb_cng_ener_fx, 9797), 1510), 8); /* Q0 */

        if ( sub(st_fx->bwidth_fx, SWB) < 0 )
        {
            idx_ener_fx = 0;
            move16();
        }

        IF ( sub(idx_ener_fx, 15) > 0 )
        {
            idx_ener_fx = 15;
            move16();
        }
        ELSE
        {
            idx_ener_fx = s_max(idx_ener_fx,0);
        }

        push_indice_fx( st_fx, IND_SHB_CNG_GAIN, idx_ener_fx, 4);
        push_indice_fx( st_fx, IND_SID_BW, 1, 1 );
        st_fx->nb_bits_tot_fx = sub(st_fx->nb_bits_tot_fx,st_fx->ind_list_fx[IND_CNG_ENV1].nb_bits);
        st_fx->ind_list_fx[IND_CNG_ENV1].nb_bits = -1;
        move16();
        push_indice_fx( st_fx, IND_UNUSED, 0, 2);
    }
    ELSE
    {
        IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
        {
            push_indice_fx( st_fx, IND_SID_BW, 0, 1 );
        }
    }

    return;
}

/*---------------------------------------------------------------------*
* shb_DTX()
*
* Decide if encoding SHB SID or not
*---------------------------------------------------------------------*/
static Word16 shb_DTX_fx(
    Encoder_State_fx *st_fx,             /* i/o: State structure                                                   */
    const Word16 *shb_speech_fx,     /* i  : SHB target signal (6-14kHz) at 16kHz       (Q0)                   */
    const Word16 *syn_12k8_16k       /* i  : ACELP core synthesis at 12.8kHz or 16kHz   (st_fx->Q_syn = 0)     */
)
{
    Word16 i;
    Word16 update_fx;
    Word16 shb_old_speech_fx[(ACELP_LOOK_12k8 + L_SUBFR + L_FRAME) * 5/4];
    Word16 *shb_new_speech_fx;
    Word32 wb_ener_fx;
    Word32 shb_ener_fx;
    Word16 log_wb_ener_fx;
    Word16 log_shb_ener_fx;
    Word16 tmp;
    Word16 exp;
    Word16 fra;
    Word16 allow_cn_step_fx=0;

    shb_new_speech_fx = shb_old_speech_fx + (ACELP_LOOK_12k8 + L_SUBFR) * 5/4;
    Copy( st_fx->old_speech_shb_fx, shb_old_speech_fx, (ACELP_LOOK_12k8 + L_SUBFR) * 5/4 );
    Copy( shb_speech_fx, shb_new_speech_fx, L_FRAME16k );
    Copy( shb_old_speech_fx + L_FRAME16k, st_fx->old_speech_shb_fx, (ACELP_LOOK_12k8 + L_SUBFR) * 5/4 );

    shb_ener_fx = L_deposit_l(0);
    FOR ( i=0; i<L_FRAME16k; i++ )
    {
        shb_ener_fx = L_mac(shb_ener_fx, shb_old_speech_fx[i], shb_old_speech_fx[i]);
    }

    shb_ener_fx = L_add(Mpy_32_16_1(shb_ener_fx, 102), 1); /* 102 in Q15, shb_ener_fx in Q1 */

    wb_ener_fx = L_deposit_l(0);
    FOR ( i=0; i<st_fx->L_frame_fx; i++ )
    {
        wb_ener_fx = L_mac(wb_ener_fx, syn_12k8_16k[i], syn_12k8_16k[i]);
    }

    wb_ener_fx = L_add(Mpy_32_16_1(wb_ener_fx, 128), 1); /* 128 in Q15, wb_ener_fx in Q1 */

    exp = norm_l(wb_ener_fx);
    fra = Log2_norm_lc(L_shl(wb_ener_fx, exp));
    exp = sub(30-1, exp);
    wb_ener_fx = Mpy_32_16(exp, fra, LG10);
    log_wb_ener_fx = round_fx(L_shl(wb_ener_fx, 10)); /* log_wb_ener_fx in Q8 */

    exp = norm_l(shb_ener_fx);
    fra = Log2_norm_lc(L_shl(shb_ener_fx, exp));
    exp = sub(30-1, exp);
    shb_ener_fx = Mpy_32_16(exp, fra, LG10);
    log_shb_ener_fx = sub(round_fx(L_shl(shb_ener_fx, 10)), 1664); /* log_shb_ener_fx in Q8 */

    IF ( st_fx->first_CNG_fx == 0 )
    {
        st_fx->mov_wb_cng_ener_fx = log_wb_ener_fx;
        move16();
        st_fx->mov_shb_cng_ener_fx = log_shb_ener_fx;
        move16();
        st_fx->last_wb_cng_ener_fx = log_wb_ener_fx;
        move16();
        st_fx->last_shb_cng_ener_fx = log_shb_ener_fx;
        move16();
    }

    if ( sub(abs_s(sub(log_wb_ener_fx, st_fx->mov_wb_cng_ener_fx)), 3072) > 0 )
    {
        allow_cn_step_fx = 1;
        move16();
    }

    IF ( sub(allow_cn_step_fx, 1) == 0 )
    {
        st_fx->mov_wb_cng_ener_fx = log_wb_ener_fx;
        move16();
        st_fx->mov_shb_cng_ener_fx = log_shb_ener_fx;
        move16();
    }
    ELSE
    {
        tmp = sub(log_wb_ener_fx, st_fx->mov_wb_cng_ener_fx); /* Q8 */
        tmp = mult(tmp, 29491); /* Q8 */
        st_fx->mov_wb_cng_ener_fx = add(st_fx->mov_wb_cng_ener_fx, tmp); /* Q8 */

        tmp = sub(log_shb_ener_fx, st_fx->mov_shb_cng_ener_fx);
        tmp = mult(tmp, 8192); /* Q8 */
        st_fx->mov_shb_cng_ener_fx = add(st_fx->mov_shb_cng_ener_fx, tmp); /* Q8 */
    }
    st_fx->shb_NO_DATA_cnt_fx = add(st_fx->shb_NO_DATA_cnt_fx, 1);

    update_fx = 0;
    move16();
    IF ( L_sub(st_fx->core_brate_fx, SID_2k40) == 0 )
    {
        test();
        test();
        IF ( st_fx->first_CNG_fx == 0 || sub(st_fx->last_vad_fx, 1) == 0 || sub(st_fx->shb_NO_DATA_cnt_fx, 100) >= 0 )
        {
            update_fx = 1;
            move16();
        }
        ELSE
        {
            IF ( st_fx->shb_cng_ini_cnt_fx > 0 )
            {
                update_fx = 1;
                move16();
                st_fx->shb_cng_ini_cnt_fx = sub(st_fx->shb_cng_ini_cnt_fx, 1);
            }
            ELSE
            {
                IF ( sub(abs_s(sub(sub(st_fx->mov_wb_cng_ener_fx, st_fx->mov_shb_cng_ener_fx), sub(st_fx->last_wb_cng_ener_fx, st_fx->last_shb_cng_ener_fx))), 768) > 0 )
                {
                    update_fx = 1;
                    move16();
                }
                ELSE
                {
                    test();
                    IF ( sub(st_fx->bwidth_fx, SWB) >= 0 && sub(st_fx->last_SID_bwidth_fx, SWB) < 0 )
                    {
                        update_fx = 1;
                        move16();
                    }
                    ELSE
                    {
                        test();
                        IF ( sub(st_fx->bwidth_fx, SWB) < 0 && sub(st_fx->last_SID_bwidth_fx, SWB) >= 0 )
                        {
                            update_fx = 1;
                            move16();
                        }
                    }
                }
            }
        }

        st_fx->last_SID_bwidth_fx = st_fx->bwidth_fx;
        move16();
    }

    IF ( sub(update_fx, 1) == 0 )
    {
        st_fx->last_wb_cng_ener_fx = st_fx->mov_wb_cng_ener_fx;
        move16();
        st_fx->last_shb_cng_ener_fx = st_fx->mov_shb_cng_ener_fx;
        move16();
        st_fx->shb_NO_DATA_cnt_fx = 0;
        move16();
    }


    return (update_fx);
}
