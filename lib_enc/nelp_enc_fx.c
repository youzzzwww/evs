/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "control.h"
#include "options.h"     /* Compilation switches */
#include "cnst_fx.h"       /* Common constants */
#include "prot_fx.h"       /* Function prototypes */
#include "rom_com_fx.h"


#include "stl.h"

/*===================================================================*/
/* FUNCTION      :  quantize_uvg_fx()                                */
/*-------------------------------------------------------------------*/
/* PURPOSE       :   This function returns the quantized gain vector */
/*                   and corresponding table indices, given the
             input unquantized gain vector                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*         _ (Word16*)  G : Un-quantized gain vector (Q0)            */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*         _ (Word16)   iG1  : UVG1CB_fx table index (Q0)            */
/*         _ (Word16*)  iG2  : UVG2CB_fx table indices (Q0)          */
/*         _ (Word16*)  quantG    : Output quantized gain vector     */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                            _ None.                                */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*                            _ None.                                */
/*===================================================================*/

void quantize_uvg_fx(Word16 *G, Word16 *iG1, Word16 *iG2, Word16 *quantG, Word16 bwidth_fx)
{
    Word16 G1[2], G2[10],L16,L16_1;
    Word16 i, j, k, ind,temp;
    Word32 mmse;
    Word16 exp,tmp,frac;
    Word32 Lacc,Lexp[2],L_tmp;
    const Word16 (*UVG1CB_fx)[2]=NULL;
    const Word16 (*UVG2CB1_fx)[5]=NULL;
    const Word16 (*UVG2CB2_fx)[5]=NULL;
    test();
    IF( sub(bwidth_fx,NB)== 0 )
    {
        UVG1CB_fx = UVG1CB_NB_FX;
        move16();/*Q13 */
        UVG2CB1_fx = UVG2CB1_NB_FX;
        move16();/*Q12 */
        UVG2CB2_fx = UVG2CB2_NB_FX;
        move16();/*Q12 */
    }
    ELSE IF( sub(bwidth_fx,WB) == 0 || sub(bwidth_fx,SWB) == 0)
    {
        UVG1CB_fx = UVG1CB_WB_FX;
        move16();/*Q13 */
        UVG2CB1_fx = UVG2CB1_WB_FX;
        move16();/*Q12 */
        UVG2CB2_fx = UVG2CB2_WB_FX;
        move16();/*Q12 */
    }


    FOR (i=0; i<2; i++)
    {
        Lacc = L_deposit_l(0);
        FOR (j=0; j<5; j++)
        {
            ind=add(shr(extract_l(L_mult(i,5)),1),j);
            Lacc=L_mac0(Lacc,G[ind],G[ind]); /*Q0 */
        }

        IF (Lacc==0)
        {
            Lacc = L_deposit_l(1); /* to avoid log10(0) */
        }

        /*G1[i] = (float) log10(sqrt(G1[i]/5)); */
        L_tmp = Mult_32_16(Lacc,13108);
        IF(L_tmp)
        {
            exp = norm_l(L_tmp);
            frac = Log2_norm_lc(L_shl(L_tmp,exp));
            exp = (30-exp-1);
            move16();/*(+1)=/2 in log */
            L_tmp = Mpy_32_16(exp,frac,4932); /*Q16 ;  0.5*log10(2) in Q15 */
            G1[i]= round_fx(L_shl(L_tmp,13)); /*Q13 */
        }
        ELSE
        {
            G1[i] = 0;
            move16();
        }

    }


    mmse = L_add(MAX_32, 0);
    *iG1=0;
    move16();
    FOR (i=0; i< UVG1_CBSIZE; i++)
    {
        L16 = shr(sub(G1[0],UVG1CB_fx[i][0]),1); /* Q12 */
        L16_1 = shr(sub(G1[1],UVG1CB_fx[i][1]),1); /* Q12 */
        Lacc = 0;
        Lacc = L_mac0(Lacc,L16,L16); /*Q24 */
        Lacc = L_mac0(Lacc,L16_1,L16_1);/*Q24       */

        IF (L_sub(Lacc,mmse)<0)
        {
            *iG1 = i;
            move16();
            mmse = Lacc;
        }

    }

    L_tmp = L_mult0(UVG1CB_fx[*iG1][0], 27213);/*Q26 */
    L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
    frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */
    L_tmp = Pow2(30, frac);
    exp = exp-30; /*move16(); */
    Lexp[0] = L_shl(L_tmp,exp +15 );


    L_tmp = L_mult0(UVG1CB_fx[*iG1][1], 27213);/*Q26 */
    L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
    frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */
    L_tmp = Pow2(30, frac);
    exp = exp-30; /*move16(); */
    Lexp[1] = L_shl(L_tmp,exp +15 );


    FOR (i=0; i<2; i++)
    {
        FOR (j=0; j<5; j++)
        {
            exp = norm_l(Lexp[i]);
            tmp = extract_h(L_shl(Lexp[i],exp));
            exp = sub(sub(30,exp),15);
            tmp = div_s(16384,tmp); /*Q(15+exp) */
            L_tmp = L_shr(L_mult0(G[i*5+j],tmp),exp+3); /*Q12 */
            G2[i*5+j] = extract_l(L_tmp); /*Q12 */
        }
    }


    FOR (i=0; i<2; i++)
    {
        mmse=MAX_32;
        iG2[i]=0;
        FOR (j=0; j<UVG2_CBSIZE; j++)
        {
            Lacc = L_deposit_l(0);
            FOR (k=0; k<5; k++)
            {
                IF (i == 0)
                {
                    /*mse += SQR(G2[i*5+k]-UVG2CB1[j][k]); */
                    ind=add(shr(extract_l(L_mult(i,5)),1),k);
                    temp=sub(G2[ind],UVG2CB1_fx[j][k]);
                    Lacc=L_mac0(Lacc,temp,temp); /*Q24 */
                }
                ELSE IF (i == 1)
                {
                    /*mse += SQR(G2[i*5+k]-UVG2CB2[j][k]); */
                    ind=add(shr(extract_l(L_mult(i,5)),1),k);
                    temp=sub(G2[ind],UVG2CB2_fx[j][k]);
                    Lacc=L_mac0(Lacc,temp,temp); /*Q24 */
                }
            }

            IF (L_sub(Lacc,mmse)<0)
            {
                mmse = Lacc;
                iG2[i] = j;
                move16();
            }
        }
    }

    dequantize_uvg_fx(*iG1, iG2, quantG, bwidth_fx ,0);

}

/*===================================================================*/
/* FUNCTION      :  normalize_arr()                                  */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Normalize array                                  */
/*                                                                   */
/*-------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS  :                                         */
/*    _ (Word16*) qf                                                 */
/*    _ (Word16*) size                                               */
/*    _ (Word16*) hdr                                                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*    _ (Word16*) arr : Normalized array                             */
/*-------------------------------------------------------------------*/

/*     _ None                                                        */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*     _ None                                                        */
/*===================================================================*/
static void normalize_arr(Word16 *arr, Word16 *qf, Word16 size, Word16 hdr)
{
    Word16 i;
    Word16 max_s = 0;

    FOR(i = 0; i < size; i++)
    {
        max_s=s_max(max_s, abs_s(arr[i]));
    }

    *qf=norm_s((Word16)max_s);
    test();
    IF((*qf == 0)&&(((Word16)max_s)==0))
    {
        *qf = 15;
        move16();
    }

    *qf = *qf-hdr;

    FOR (i = 0; i < size; i++)
    {
        arr[i] = shl(arr[i], *qf);
        move16(); /* saturation can occur here */
    }

    return;
}


/*===================================================================*/
/* FUNCTION      :  nelp_encoder_fx()                                */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  NELP encoder                                     */
/*                                                                   */
/*-------------------------------------------------------------------*/
/* GLOBAL INPUT ARGUMENTS  :                                         */
/*    _ (Struct)   st     : encoder state                            */
/*    _ (Word16[]) in_fx  : residual signal (qIn)                    */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*    _ (Word16[]) exc_fx : NELP quantized excitation signal (qIn)   */
/*    _ (Word16)   qIn1   : input/output qformat                     */
/*-------------------------------------------------------------------*/

/*    _ (Word16[])  shape1_filt_mem_fx : filter memory (Q0)          */
/*    _ (Word16[])  shape2_filt_mem_fx : filter memory (Q0)          */
/*    _ (Word16[])  shape3_filt_mem_fx : filter memory (Q0)          */
/*    _ (Word16[])  bp1_filt_mem_wb_fx : filter memory (Q0)          */
/*    _ (Word16[])  txlpf1_filt1_mem_fx : filter memory (Q0)         */
/*    _ (Word16[])  txlpf1_filt1_mem_fx : filter memory (Q0)         */
/*    _ (Word16[])  txhpf1_filt2_mem_fx : filter memory (Q-1)        */
/*    _ (Word16[])  txlpf1_filt2_mem_fx : filter memory (Q0)         */
/*    _ (Word16)    nelp_gain_mem_fx    : gain memory (Q0)           */
/*    _ (Word16)    nelp_enc_seed       :                            */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*     _ None                                                        */
/*===================================================================*/

void nelp_encoder_fx(
    Encoder_State_fx *st_fx,         /* i/o: encoder state                      */
    Word16 *in_fx,         /* i  : residual signal                    */
    Word16 *exc_fx,         /* o  : NELP quantized excitation signal   */
    Word16 *qIn1
    ,Word16 reduce_gains
)
{
    Word16 i,j;
    Word16 *ptr_fx = exc_fx;
    Word16 lag = 25; /* to cover 25*9 + 31 */
    Word16 sqrt_inv_lag = 6554; /* sqrt(1/lag) in Q15 */
    Word16 sqrt_inv_lframe_lag = 5885; /* sqrt(1/(L_FRAME-lag*9)) */
    Word16 Gains_fx[10], gain_fac_fx;
    Word16 iG1_fx, iG2_fx[2];
    Word16 fid;
    Word16 fdbck_fx;
    Word32 var_dB_fx;
    Word32 E1_fx=0, EL1_fx=0, EH1_fx=0, E2_fx=0, E3_fx=0, EL2_fx=0, EH2_fx=0;
    Word32 RL_fx=0, RH_fx=0;
    Word16 R_fx=0;
    Word16 filtRes_fx[L_FRAME];
    Word16 ptr_tmp_fx[L_FRAME];

    Word16 qE1=0, qE2=0, qE3=0, qEL1=0, qEL2=0, qEH1=0, qEH2=0;
    Word16 qIn=0, qGain=0, qf=0, qf1=0, qNelpGain=0;
    Word16 exp1, exp2, tmp1, tmp2;
    Word16 f_Noise, etmp, e_Noise;
    Word16 max1=0;
    Word32 l_nelp_gain_mem;
    Word32 Ltemp = 0, Ltemp1 = 0, L_tmp = 0, L_const_1;
    Word16 BP1_ORDER;
    Word16 rf_flag;

    rf_flag = st_fx->rf_mode;

    qIn = *qIn1;
    move16();
    test();
    IF (sub(st_fx->bwidth_fx,NB) == 0)
    {
        IF (st_fx->last_nelp_mode_fx != 1)
        {
            BP1_ORDER = 7;
            move16();
            set32_fx(st_fx->bp1_filt_mem_nb_fx, 0, BP1_ORDER*2);
            st_fx->qprevGain_fx = 0;
            move16();
        }
    }
    ELSE IF (sub(st_fx->bwidth_fx,WB) == 0 || sub(st_fx->bwidth_fx,SWB) == 0)
    {
        IF (st_fx->last_nelp_mode_fx != 1)
        {
            BP1_ORDER =4;
            move16();
            set16_fx(st_fx->bp1_filt_mem_wb_fx, 0, BP1_ORDER*2);
        }
    }

    IF (st_fx->last_nelp_mode_fx != 1)
    {
        test();
        IF (st_fx->bwidth_fx == WB || sub(st_fx->bwidth_fx,SWB) == 0)
        {
            set16_fx(st_fx->shape1_filt_mem_fx, 0, 20);
            set16_fx(st_fx->shape2_filt_mem_fx, 0, 20);
            set16_fx(st_fx->shape3_filt_mem_fx, 0, 20);
            set16_fx(st_fx->txlpf1_filt1_mem_fx, 0, 20);
            set16_fx(st_fx->txlpf1_filt2_mem_fx, 0, 20);
            set16_fx(st_fx->txhpf1_filt1_mem_fx, 0, 20);
            set16_fx(st_fx->txhpf1_filt2_mem_fx, 0, 20);
            st_fx->qprevIn_fx = 0;
            move16();
            st_fx->qprevGain_fx = 0;
            move16();
        }
    }

    /* Start Unvoiced/NELP Processing */
    test();
    IF ( sub(st_fx->bwidth_fx,WB) == 0 || sub(st_fx->bwidth_fx,SWB) == 0)
    {
        qE1 = qIn;
        move16();
        E1_fx = L_deposit_l(0);
        FOR (i=0 ; i<L_FRAME; i++)
        {
            E1_fx = L_mac0(E1_fx, in_fx[i], in_fx[i]); /*Q(qE1+qE1) */
        }

        qE1 = shl(qE1, 1);;

        qf = qIn;
        move16();
        Scale_sig(st_fx->txlpf1_filt1_mem_fx, 20, (qf-st_fx->qprevIn_fx));
        pz_filter_sp_fx(txlpf1_num_coef_fx,txlpf1_den_coef_fx, in_fx, filtRes_fx, st_fx->txlpf1_filt1_mem_fx,10,10, L_FRAME, 3);/*1 = (16-qformat of shape1 cofficient) */

        qEL1 = qf;
        move16();
        EL1_fx = L_deposit_l(0);
        FOR (i=0 ; i<L_FRAME; i++)
        {
            EL1_fx = L_mac0(EL1_fx, filtRes_fx[i], filtRes_fx[i]); /*Q(2*qIn) */
        }
        qEL1 = shl(qEL1, 1);

        qf = qIn;
        move16();
        Scale_sig(st_fx->txhpf1_filt1_mem_fx, 20, qf-st_fx->qprevIn_fx);
        pz_filter_sp_fx(txhpf1_num_coef_fx,txhpf1_den_coef_fx, in_fx, filtRes_fx, st_fx->txhpf1_filt1_mem_fx,10,10, L_FRAME, 3);/*1 = (16-qformat of shape1 cofficient) */
        st_fx->qprevIn_fx = qf;
        move16();

        qEH1 = qf;
        move16();
        EH1_fx = L_deposit_l(0);
        FOR (i=0 ; i<L_FRAME; i++)
        {
            EH1_fx = L_mac0(EH1_fx, filtRes_fx[i], filtRes_fx[i]); /*Q(2*qEH1) */
        }
        qEH1 = 2*qEH1;
        move16();

    }

    qGain = qIn;
    move16();
    qGain = 2*qGain;
    move16();

    FOR (i=0; i<9; i++)
    {
        Ltemp = L_deposit_l(0);
        FOR (j = (Word16)(i*lag) ; j<(Word16)((i+1)*lag); j++)
        {
            Ltemp = L_mac0(Ltemp, in_fx[j], in_fx[j]); /*Q(2*qGain) */
        }

        /*Gains[i] = (float) sqrt(Gains[i]/lag); */
        IF(Ltemp != 0)
        {
            exp1 = norm_l(Ltemp);
            tmp1 = extract_h(L_shl(Ltemp, exp1));/*2*qGain+exp1-16 */
            exp1 = sub(exp1, 30-qGain); /* */

            tmp1 = div_s(16384, tmp1);/*14-2*qGain-exp1+16 */
            L_tmp = L_deposit_h(tmp1);
            L_tmp = Isqrt_lc(L_tmp, &exp1);
            L_tmp = Mult_32_16(L_tmp, sqrt_inv_lag);
            Ltemp = L_shl(L_tmp, sub(exp1, 12));/*Q3 */
        }

        Gains_fx[i] = round_fx(Ltemp);
    }


    Ltemp = L_deposit_l(0);
    FOR (j = (Word16)(i*lag) ; j<L_FRAME; j++)
    {
        Ltemp = L_mac0(Ltemp, in_fx[j], in_fx[j]); /*Q(2*qGain) */
    }

    /*Gains[i] = (float) sqrt(Gains[i]/(L_FRAME-(lag*i))); */
    IF(Ltemp != 0)
    {
        exp1 = norm_l(Ltemp);
        tmp1 = extract_h(L_shl(Ltemp, exp1));
        exp1 = sub(exp1, 30-qGain); /* */

        tmp1 = div_s(16384, tmp1);
        L_tmp = L_deposit_h(tmp1);
        L_tmp = Isqrt_lc(L_tmp, &exp1);
        L_tmp = Mult_32_16(L_tmp, sqrt_inv_lframe_lag);
        Ltemp = L_shl(L_tmp, sub(exp1, 12));
    }

    Gains_fx[i] = round_fx(Ltemp);

    IF ( sub(reduce_gains,1) == 0)
    {
        FOR (i=0; i<10; i++)
        {
            Gains_fx[i] = mult( Gains_fx[i], 19661 );
        }
    }


    qGain = 3;
    move16();
    IF (st_fx->last_nelp_mode_fx != 1) /* if prev frame was not NELP then init mem*/
    {
        st_fx->nelp_gain_mem_fx = Gains_fx[0];
        move16();
        qNelpGain = qGain;
        move16();
    }

    /* tmp = (float) (20.0 * (log10 (Gains[0]) - log10 (st->nelp_gain_mem) ) ); */
    /* var_dB = tmp * tmp; */
    L_tmp = L_deposit_l(Gains_fx[0]);
    exp2 = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, exp2);/*15+qEL1-qEL2-exp1+exp2 */
    exp2 = 30-exp2-qGain;
    move16();
    tmp1 = Log2_norm_lc(L_tmp);
    Ltemp = Mpy_32_16(exp2, tmp1,9864); /*log(2) in Q13 format = Q0 format */
    /*tmp1 = round_fx(L_shl(Ltemp,12)); Q12 */

    L_tmp = L_deposit_l(st_fx->nelp_gain_mem_fx); /*Q0 */
    exp2 = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, exp2);/*15+qEL1-qEL2-exp1+exp2 */
    exp2 = 30-exp2-qNelpGain;
    move16();
    tmp2 = Log2_norm_lc(L_tmp);
    Ltemp1 = Mpy_32_16(exp2, tmp2,9864); /*log(2) in Q13 format = Q0 format */
    Ltemp1 = L_sub(Ltemp, Ltemp1);/*Q16 */
    Ltemp = Mult_32_16(Ltemp1, 20480);/*Q11 (20 in Q10) */
    L_tmp = L_shl(Ltemp,12);/*Q23 */
    var_dB_fx = Mult_32_32(L_tmp, L_tmp);/*Q15 */

    FOR (i = 1; i < 10; i++)
    {
        L_tmp = L_deposit_l(Gains_fx[i]);
        exp2 = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, exp2);/*15+qEL1-qEL2-exp1+exp2 */
        exp2 = 30-exp2-qGain;
        move16();
        tmp1 = Log2_norm_lc(L_tmp);
        Ltemp = Mpy_32_16(exp2, tmp1,9864); /*log(2) in Q13 format = Q0 format */

        L_tmp = L_deposit_l(Gains_fx[i-1]); /*Q0 */
        exp2 = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, exp2);/*15+qEL1-qEL2-exp1+exp2 */
        exp2 = 30-exp2-qGain;
        move16();
        tmp2 = Log2_norm_lc(L_tmp);
        Ltemp1 = Mpy_32_16(exp2, tmp2,9864); /*log(2) in Q13 format = Q0 format */
        Ltemp1 = L_sub(Ltemp, Ltemp1);/*Q16 */
        Ltemp = Mult_32_16(Ltemp1, 20480);/*Q11 (20 in Q10) */
        L_tmp = L_shl(Ltemp,12);/*Q23 */
        L_tmp = Mult_32_32(L_tmp, L_tmp);/*Q15 */
        var_dB_fx = L_add(L_tmp, var_dB_fx);/*Q15 */
    }

    IF (st_fx->last_nelp_mode_fx!=1)
    {
        /*var_dB *= 0.111f; */
        var_dB_fx = Mult_32_16(var_dB_fx, 3637); /*0.111 in Q15 */
    }
    ELSE
    {
        /*var_dB *= 0.1f; */
        var_dB_fx = Mult_32_16(var_dB_fx, 3277); /*0.1 in Q15 */
    }

    max1 = 0;
    move16();
    FOR(i = 0; i < 10; i++)
    {
        max1=s_max(max1, abs_s(Gains_fx[i]));
    }

    qf=norm_s((Word16)max1);
    test();
    IF((qf == 0)&&(((Word16)max1)==0))
    {
        qf= 15;
        move16();
    }
    qf = sub(qf,1);
    qGain = add(qGain,qf);

    Scale_sig(Gains_fx, 10, qf);

    L_tmp = L_sub(var_dB_fx, 655360); /* 20 in Q15 */
    Ltemp = L_shr_r(L_tmp, 2);/*Q15 */
    {
        /*exp  = pow(2, x*log2(e)) */
        L_tmp = Mult_32_16(Ltemp, 23637);	/*15 + 14 -15 ->Q14 */
        L_tmp = L_shl(L_tmp, 2);	/*Q16 */
        f_Noise = L_Extract_lc(L_tmp, &e_Noise);			/*Q16 */
        etmp = extract_l(Pow2(14, f_Noise));			/* Put 14 as exponent */
        e_Noise = sub(e_Noise, 14);						/* Retreive exponent of etmp */

        IF(e_Noise > 0)
        {
            L_tmp = L_shl(etmp,e_Noise);			/* Result in Q30 */
            L_tmp = L_add(1, L_tmp);

            exp1 = norm_l(L_tmp);
            tmp1 = extract_h(L_shl(L_tmp, exp1));/*exp1-16 */
            tmp1 = div_s(16384, tmp1);/*14-(exp1-16)-> 30+15-exp1 */
            fdbck_fx = mult(26870, tmp1);/*45-exp1+15-15=>45-exp1 */
            fdbck_fx = shr_r(fdbck_fx, 14);
            exp1 = sub(31,exp1);
        }
        ELSE
        {
            L_tmp = L_shl(etmp,add(e_Noise, 14));			/* Result in Q30 */
            L_tmp = L_add(16384, L_tmp);

            exp1 = norm_l(L_tmp);
            tmp1 = extract_h(L_shl(L_tmp, exp1));/*14+exp1-16 */
            tmp1 = div_s(16384, tmp1);/*14-(14+exp1-16)-> 16+15-exp1 */
            fdbck_fx = mult(26870, tmp1);/*31-exp1+15-15=>31-exp1 */
            exp1 = sub(31,exp1);
        }
    }

    IF(exp1 == 31)
    {
        L_const_1=0x7fffffff;
        move32();
    }
    ELSE
    {
        L_const_1 = L_shl(1, exp1);
    }

    l_nelp_gain_mem =  L_deposit_l(st_fx->nelp_gain_mem_fx);
    IF(sub(qNelpGain,qGain) != 0)
    {
        l_nelp_gain_mem =  L_shl(l_nelp_gain_mem, sub(qGain, qNelpGain));
    }

    FOR (i = 0; i < 10; i++)
    {
        /*Gains[i] = (float)((1.0f - fdbck) * Gains[i] + fdbck * st->nelp_gain_mem); */
        L_tmp = L_sub(L_const_1, L_deposit_l(fdbck_fx));/*31-exp1 */
        L_tmp = Mult_32_16(L_tmp, Gains_fx[i]);/*exp1+qGain-15=>exp1-15+qGain */
        Ltemp1 = Mult_32_16(l_nelp_gain_mem, fdbck_fx);/*exp1+qGain-15 */
        L_tmp = L_add(L_tmp, Ltemp1);
        L_tmp = L_shr_r(L_tmp, (exp1-15));
        Gains_fx[i] = round_fx(L_shl(L_tmp,16));
        move16();
        l_nelp_gain_mem = L_tmp;
    }

    st_fx->nelp_gain_mem_fx = round_fx(L_shl(l_nelp_gain_mem,16));

    Scale_sig(&st_fx->nelp_gain_mem_fx, 1, -qGain);
    Scale_sig(Gains_fx, 10, -qGain);
    qGain =0;
    move16();

    quantize_uvg_fx(Gains_fx, &iG1_fx, iG2_fx, Gains_fx,  st_fx->bwidth_fx);

    IF( sub(rf_flag,1) == 0 )
    {
        st_fx->rf_indx_nelp_iG1[0] = iG1_fx;
        st_fx->rf_indx_nelp_iG2[0][0] = iG2_fx[0];
        st_fx->rf_indx_nelp_iG2[0][1] = iG2_fx[1];
    }
    ELSE
    {
        push_indice_fx( st_fx, IND_IG1, iG1_fx, 5);
        push_indice_fx( st_fx, IND_IG2A, iG2_fx[0], 6 );
        push_indice_fx( st_fx, IND_IG2B, iG2_fx[1], 6 );
    }

    test();
    IF (sub(st_fx->bwidth_fx,WB) == 0  || sub(st_fx->bwidth_fx,SWB) == 0)
    {
        gain_fac_fx = 19005;
        move16();/* 1.16f in Q14 */
    }
    ELSE
    {
        gain_fac_fx = 22446;
        move16(); /* 1.37f in Q14 */
    }

    /* Normalize Gains_fx[10] with headroom 4 */
    /* This fills up qGain with some new value */
    normalize_arr(Gains_fx, &qGain, 10, 4);

    generate_nelp_excitation_fx(&(st_fx->nelp_enc_seed_fx), Gains_fx, ptr_fx, gain_fac_fx);
    test();
    IF (sub(st_fx->bwidth_fx,WB) == 0  || sub(st_fx->bwidth_fx,SWB) == 0)
    {
        BP1_ORDER = 4;
        Scale_sig(st_fx->bp1_filt_mem_wb_fx, BP1_ORDER*2, qGain-st_fx->qprevGain_fx);/*qf-qAdj */
        pz_filter_sp_fx(bp1_num_coef_wb_fx,bp1_den_coef_wb_fx, ptr_fx, ptr_tmp_fx, st_fx->bp1_filt_mem_wb_fx, BP1_ORDER, BP1_ORDER, L_FRAME, 2);
        Copy(ptr_tmp_fx,ptr_fx,L_FRAME);
    }
    ELSE IF (sub(st_fx->bwidth_fx,NB) == 0)
    {
        BP1_ORDER = 7;
        move16();
        Scale_sig32(st_fx->bp1_filt_mem_nb_fx, BP1_ORDER*2, (qGain - st_fx->qprevGain_fx));
        pz_filter_dp_fx(bp1_num_coef_nb_fx_order7, bp1_den_coef_nb_fx_order7, ptr_fx, ptr_tmp_fx, st_fx->bp1_filt_mem_nb_fx, BP1_ORDER, BP1_ORDER, L_FRAME, (sub(16,BP1_COEF_NB_QF_ORDER7)));
        Copy(ptr_tmp_fx,ptr_fx,L_FRAME);

        Scale_sig(ptr_fx, L_FRAME, -1); /* bring exc to qgain-1         */
        *qIn1 = qGain-1;  /* use this temp only in the parent */
    }

    E3_fx = L_deposit_l(0);
    FOR (i=0 ; i<L_FRAME; i++)
    {
        E3_fx = L_mac(E3_fx, ptr_fx[i], ptr_fx[i]); /*Q1 */
    }
    qE3= 2*qGain+1;
    move16();
    test();
    IF (st_fx->bwidth_fx == WB|| sub(st_fx->bwidth_fx,SWB) == 0)
    {
        Scale_sig(st_fx->shape1_filt_mem_fx, 20, (qGain-st_fx->qprevGain_fx));
        pz_filter_sp_fx(shape1_num_coef_fx,shape1_den_coef_fx, ptr_fx, ptr_tmp_fx, st_fx->shape1_filt_mem_fx,10,10, L_FRAME, 1);/*1 = (16-qformat of shape1 cofficient) */

        Copy(ptr_tmp_fx,ptr_fx,L_FRAME);

        qf = qGain;
        move16();
        E2_fx = L_deposit_l(0);
        FOR (i=0 ; i<L_FRAME; i++)
        {
            Ltemp = L_mult0(ptr_fx[i], ptr_fx[i]); /*Q(2*qE2+1) */
            Ltemp = L_shr_r(Ltemp, 4);
            E2_fx = L_add(E2_fx, Ltemp);
        }
        qE2 = 2*qf-4;
        move16();

        test();
        IF(E1_fx==0)
        {
            R_fx=0;
            move16();
        }
        ELSE IF((E2_fx==0)&&(E1_fx!=0))
        {
            exp1 = norm_l(E1_fx);
            tmp1 = extract_h(L_shl(E1_fx, exp1));/*qE1+exp1-16 */
            tmp1 = div_s(16384, tmp1);/*14-(qE1+exp1-16)-> 30-qE1-exp1 */
            exp1 = sub(exp1, 30-qE1);

            L_tmp = L_deposit_h(tmp1);
            L_tmp = Isqrt_lc(L_tmp, &exp1);
            Ltemp = L_shl(L_tmp, sub(exp1, 12));
            R_fx = round_fx(Ltemp);
        }
        ELSE
        {
            exp1 = norm_l(E1_fx);
            tmp1 = extract_h(L_shl(E1_fx, exp1));/*qE1+exp1-16 */
            tmp1 = div_s(16384, tmp1);/*14-(qE1+exp1-16)-> 30-qE1-exp1 */
            L_tmp = Mult_32_16(E2_fx, tmp1);/*qE2+30-qE1-exp1-15=>15+qE2-qE1-exp1 */

            exp2 = norm_l(L_tmp);
            L_tmp = L_shl(L_tmp, exp2);/*15+qE2-qE1-exp1+exp2 */
            exp2 = 30-(15+qE2-qE1-exp1+exp2);
            move16();
            L_tmp = Isqrt_lc(L_tmp, &exp2);/*Q(31+exp2) */
            R_fx = round_fx(L_tmp);
            exp1 = 31-exp2-16-7;
            move16();
        }

        FOR (i=0; i<L_FRAME; i++)
        {
            Ltemp = L_mult0(R_fx, ptr_fx[i]);
            Ltemp = L_shr_r(Ltemp, exp1);
            filtRes_fx[i] = round_fx(L_shl(Ltemp,16));
        }

        qf1 = qGain;
        move16();
        Scale_sig(st_fx->txlpf1_filt2_mem_fx, 20, (qf1-st_fx->qprevGain_fx));

        pz_filter_sp_fx(txlpf1_num_coef_fx,txlpf1_den_coef_fx, filtRes_fx, ptr_tmp_fx, st_fx->txlpf1_filt2_mem_fx,10,10, L_FRAME, 3);/*1 = (16-qformat of shape1 cofficient) */
        Copy(ptr_tmp_fx,filtRes_fx,L_FRAME);

        qEL2 = qf1;
        move16();
        EL2_fx = L_deposit_l(0);
        FOR (i=0; i<L_FRAME; i++)
        {
            EL2_fx = L_mac0(EL2_fx, filtRes_fx[i], filtRes_fx[i]); /*Q(2*qEL2) */
        }
        qEL2 = 2*qEL2;
        move16();

        FOR (i=0; i<L_FRAME; i++)
        {
            Ltemp = L_mult0(R_fx, ptr_fx[i]);
            Ltemp = L_shr_r(Ltemp, exp1);
            filtRes_fx[i] = round_fx(L_shl(Ltemp,16));
        }

        qf = qGain;
        move16();
        Scale_sig(st_fx->txhpf1_filt2_mem_fx, 20, (qf-st_fx->qprevGain_fx));
        pz_filter_sp_fx(txhpf1_num_coef_fx,txhpf1_den_coef_fx, filtRes_fx, ptr_tmp_fx, st_fx->txhpf1_filt2_mem_fx,10,10, L_FRAME, 3);/*1 = (16-qformat of shape1 cofficient) */

        Copy(ptr_tmp_fx,filtRes_fx,L_FRAME);

        qEH2 = qf;
        move16();
        EH2_fx = L_deposit_l(0);
        FOR (i=0; i<L_FRAME; i++)
        {
            EH2_fx = L_mac0(EH2_fx, filtRes_fx[i], filtRes_fx[i]); /*Q(2*qEH2) */
        }
        qEH2 = 2*qEH2;
        move16();
        IF(EL2_fx==0)
        {
            exp2 = norm_l(EL1_fx);
            L_tmp = L_shl(EL1_fx, exp2);
            exp2 = 30-exp2-qEL1;
            move16();
            tmp1 = Log2_norm_lc(L_tmp);
            Ltemp = Mpy_32_16(exp2, tmp1,9864); /*10*log(2) in Q15 format = Q0 format */
            tmp1 = round_fx(L_shl(Ltemp,12));/* Q12 */
            RL_fx = L_mult0(tmp1, 10);
        }
        ELSE
        {
            exp1 = norm_l(EL2_fx);
            tmp1 = extract_h(L_shl(EL2_fx, exp1));/*qEL2+exp1-16 */
            tmp1 = div_s(16384, tmp1);/*14-(qEL2+exp1-16)-> 30-qEL2-exp1 */
            L_tmp = Mult_32_16(EL1_fx, tmp1);/*qEL1+30-qEL2-exp1-15=>15+qE1-qEL2-exp1 */

            exp2 = norm_l(L_tmp);
            L_tmp = L_shl(L_tmp, exp2);/*15+qEL1-qEL2-exp1+exp2 */
            exp2 = 30-(30+qEL1-qEL2-exp1+exp2);
            move16();
            tmp1 = Log2_norm_lc(L_tmp);
            Ltemp = Mpy_32_16(exp2, tmp1,9864); /*10*log(2) in Q15 format = Q0 format */
            tmp1 = round_fx(L_shl(Ltemp,12));/* Q12 */
            RL_fx = L_mult0(tmp1, 10);
        }

        IF(EH2_fx==0)
        {
            exp2 = norm_l(EH2_fx);
            L_tmp = L_shl(EH2_fx, exp2);
            exp2 = 30-exp2-qEH2;
            move16();
            tmp1 = Log2_norm_lc(L_tmp);
            Ltemp = Mpy_32_16(exp2, tmp1,9864); /*10*log(2) in Q13 format = Q0 format */
            tmp1 = round_fx(L_shl(Ltemp,12));/* Q12 */
            RH_fx = L_mult0(tmp1, 10);
        }
        ELSE
        {
            exp1 = norm_l(EH2_fx);
            tmp1 = extract_h(L_shl(EH2_fx, exp1));/*qEH2+exp1-16 */
            tmp1 = div_s(16384, tmp1);/*14-(qEH2+exp1-16)-> 30-qEH2-exp1 */
            L_tmp = Mult_32_16(EH1_fx, tmp1);/*15+qEH1-qEH2-exp1 */

            exp2 = norm_l(L_tmp);
            L_tmp = L_shl(L_tmp, exp2);/*15+qEH1-qEH2-exp1+exp2 */
            exp2 = 30-(30+qEH1-qEH2-exp1+exp2);
            move16();
            tmp1 = Log2_norm_lc(L_tmp);
            Ltemp = Mpy_32_16(exp2, tmp1,9864); /*10*log(2) in Q13 format = Q0 format */
            tmp1 = round_fx(L_shl(Ltemp,12));/* Q12 */
            RH_fx = L_mult0(tmp1, 10);
        }

        fid = 0;
        move16();
        IF (L_sub(RL_fx, -12288) < 0)  /* -3 in Q12 */
        {
            fid = 1;
            move16();
        }
        ELSE IF (L_sub(RH_fx, -12288) < 0) /* -3 in Q12 */
        {
            fid = 2;
            move16();
        }
        SWITCH(fid)
        {
        case 1:
            /* Update other filter memory */
            Scale_sig(st_fx->shape3_filt_mem_fx, 20, (qGain-st_fx->qprevGain_fx));
            pz_filter_sp_fx(shape3_num_coef_fx,shape3_den_coef_fx, ptr_fx, filtRes_fx, st_fx->shape3_filt_mem_fx,10,10, L_FRAME, 1);/*1 = (16-qformat of shape1 cofficient) */


            /* filter the residual to desired shape */
            Scale_sig(st_fx->shape2_filt_mem_fx, 20, (qGain-st_fx->qprevGain_fx));
            pz_filter_sp_fx(shape2_num_coef_fx,shape2_den_coef_fx, ptr_fx, ptr_tmp_fx, st_fx->shape2_filt_mem_fx,10,10, L_FRAME, 1);/*1 = (16-qformat of shape1 cofficient) */

            Copy(ptr_tmp_fx,ptr_fx,L_FRAME);

            BREAK;
        case 2:
            /* Update other filter memory */
            Scale_sig(st_fx->shape2_filt_mem_fx, 20, (qGain-st_fx->qprevGain_fx));
            pz_filter_sp_fx(shape2_num_coef_fx,shape2_den_coef_fx, ptr_fx, filtRes_fx, st_fx->shape2_filt_mem_fx,10,10, L_FRAME, 1);/*1 = (16-qformat of shape1 cofficient) */

            /* filter the residual to desired shape */
            Scale_sig(st_fx->shape3_filt_mem_fx, 20, (qGain-st_fx->qprevGain_fx));
            pz_filter_sp_fx(shape3_num_coef_fx,shape3_den_coef_fx, ptr_fx, ptr_tmp_fx, st_fx->shape3_filt_mem_fx,10,10, L_FRAME, 1);/*1 = (16-qformat of shape1 cofficient) */

            Copy(ptr_tmp_fx,ptr_fx,L_FRAME);

            BREAK;
        default:
            Scale_sig(st_fx->shape2_filt_mem_fx, 20, (qGain-st_fx->qprevGain_fx));
            pz_filter_sp_fx(shape2_num_coef_fx,shape2_den_coef_fx, ptr_fx, filtRes_fx, st_fx->shape2_filt_mem_fx,10,10, L_FRAME, 1);/*1 = (16-qformat of shape1 cofficient) */

            Scale_sig(st_fx->shape3_filt_mem_fx, 20, (qGain-st_fx->qprevGain_fx));
            pz_filter_sp_fx(shape3_num_coef_fx,shape3_den_coef_fx, ptr_fx, filtRes_fx, st_fx->shape3_filt_mem_fx,10,10, L_FRAME, 1);/*1 = (16-qformat of shape1 cofficient) */

            BREAK;
        }

        qE2 = qGain;
        move16();

        E2_fx = L_deposit_l(0);
        FOR (i=0 ; i<L_FRAME; i++)
        {
            Ltemp = L_mult0(ptr_fx[i], ptr_fx[i]); /*Q(2*qE2+1) */
            Ltemp = L_shr_r(Ltemp, 4);
            E2_fx = L_add(E2_fx, Ltemp);
        }
        qE2 = (2*qE2)-4;
        move16();
        test();
        IF(E3_fx==0)
        {
            R_fx=0;
            move16();
        }
        ELSE IF((E2_fx==0)&&(E3_fx!=0))
        {
            exp1 = norm_l(E3_fx);
            tmp1 = extract_h(L_shl(E3_fx, exp1));/*qE3+exp1-16 */
            tmp1 = div_s(16384, tmp1);/*14-(qE3+exp1-16)-> 30-qE3-exp1 */
            exp1 = sub(exp1, 30-qE3);

            L_tmp = L_deposit_h(tmp1);
            L_tmp = Isqrt_lc(L_tmp, &exp1);
            Ltemp = L_shl(L_tmp, sub(exp1, 12));
            R_fx = round_fx(Ltemp);
        }
        ELSE
        {
            exp1 = norm_l(E3_fx);
            tmp1 = extract_h(L_shl(E3_fx, exp1));/*qE3+exp1-16 */
            tmp1 = div_s(16384, tmp1);/*14-(qE3+exp1-16)-> 30-qE3-exp1 */
            L_tmp = Mult_32_16(E2_fx, tmp1);/*qE2+30-qE3-exp1-15=>15+qE2-qE3-exp1 */

            exp2 = norm_l(L_tmp);
            L_tmp = L_shl(L_tmp, exp2);/*15+qE2-qE3-exp1+exp2 */
            exp2 = 30-(15+qE2-qE3-exp1+exp2);
            move16();
            L_tmp = Isqrt_lc(L_tmp, &exp2);/*Q(31+exp2) */
            R_fx = round_fx(L_tmp);
            exp1 = 31-exp2-16-7;
            move16();
        }

        FOR (i=0; i<L_FRAME; i++)
        {
            L_tmp = L_mult0(R_fx, ptr_fx[i]);
            L_tmp = L_shr_r(L_tmp, exp1+1);
            ptr_fx[i] = round_fx(L_shl(L_tmp,16));
        }
        *qIn1 = qGain-1;
        move16();

        if(sub(rf_flag,1)==0)
        {
            st_fx->rf_indx_nelp_fid[0] = fid;
        }
        else
        {
            push_indice_fx( st_fx, IND_NELP_FID, fid, 2 );
        }
    }

    st_fx->qprevGain_fx = qGain;
    move16();

    FOR (i = 0; i < L_FRAME; i++)
    {
        exc_fx[i] = ptr_fx[i];
        move16();
    }

    return;
}


