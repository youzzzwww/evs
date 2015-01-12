/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>

#include "options.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "patch.h"
#include "log2.h"
#include "stl.h"

#define WARP_OS_RATE 8
#define LL 256
#define LL_OS (WARP_OS_RATE*LL)
#define OSLENGTH 12
#define CUTFREE_ABS_RANGE 6
#define CUTFREE_REL_RANGE 0.25
#define CUTFREE_ABS_RANGE_Q3 48
#define CUTFREE_REL_RANGE_Q2 1
#define WI_THRESHLD 0.8
#define WI_SAMPLE_THLD 20
#define ERB_CBSIZE1 64
#define ERB_CBSIZE2 64
#define P_CBSIZE 64

/*#define _POLY1(x, c)    ((c)[0] * (x) + (c)[1]) */
/*#define _POLY2(x, c)    (_POLY1((x), (c)) * (x) + (c)[2]) */
/*#define _POLY3(x, c)    (_POLY2((x), (c)) * (x) + (c)[3]) */

/*-------------------------------------------------------------------*
* DTFS_new_fx()
*
* DTFS structure initialization.
*-------------------------------------------------------------------*/
DTFS_STRUCTURE_FX* DTFS_new_fx(void)
{

    Word16 i ;

    DTFS_STRUCTURE_FX* dtfs_fx = NULL;
    dtfs_fx = (DTFS_STRUCTURE_FX *) calloc(1,sizeof(DTFS_STRUCTURE_FX));
    dtfs_fx->lag_fx = 0 ;
    move16();
    dtfs_fx->nH_fx=0;
    move16();
    dtfs_fx->nH_4kHz_fx=0;
    move16();
    dtfs_fx->upper_cut_off_freq_of_interest_fx=3300;
    move16();
    dtfs_fx->upper_cut_off_freq_fx=4000;
    move16();
    dtfs_fx->Q = 0;
    move16();

    FOR(i=0; i<MAXLAG_WI; i++)
    {
        dtfs_fx->a_fx[i] = 0;
        move16();
        dtfs_fx->b_fx[i] = 0;
        move16();
    }

    return dtfs_fx;    /* o: DTFS structure_fx  */

}
/*-------------------------------------------------------------------*
* DTFS_copy_fx()
*
* Copy from one DTFS STRUCTURE to another.
*-------------------------------------------------------------------*/

void DTFS_copy_fx(
    DTFS_STRUCTURE_FX *Xout_fx,  /* o: DTFS structure  */
    DTFS_STRUCTURE_FX Xinp_fx    /* i: DTFS structure  */
)

{

    Word16 k;
    FOR(k=0; k<MAXLAG_WI; k++)
    {
        Xout_fx->a_fx[k]=Xinp_fx.a_fx[k];
        move16();
    }

    FOR(k=0; k<MAXLAG_WI; k++)
    {
        Xout_fx->b_fx[k]=Xinp_fx.b_fx[k];
        move16();
    }

    Xout_fx->lag_fx=Xinp_fx.lag_fx;
    move16();
    Xout_fx->nH_fx=Xinp_fx.nH_fx;
    move16();
    Xout_fx->nH_4kHz_fx=Xinp_fx.nH_4kHz_fx;
    move16();
    Xout_fx->upper_cut_off_freq_of_interest_fx=Xinp_fx.upper_cut_off_freq_of_interest_fx;
    move16();
    Xout_fx->upper_cut_off_freq_fx=Xinp_fx.upper_cut_off_freq_fx;
    move16();
    Xout_fx->Q = Xinp_fx.Q;
    move16();
}


/*-------------------------------------------------------------------*
* DTFS_sub_fx()
*
* Subtract one DTFS STRUCTURE from another.
*-------------------------------------------------------------------*/
DTFS_STRUCTURE_FX DTFS_sub_fx(

    DTFS_STRUCTURE_FX X1,  /* i: DTFS input 1 */
    DTFS_STRUCTURE_FX X2   /* i: DTFS input 2 */
)
{
    DTFS_STRUCTURE_FX tmp ;
    Word16 i ,sft, tmp_loop;
    sft = sub(X1.Q,X2.Q);

    IF(sft>0)
    {
        tmp_loop = shr(X1.lag_fx,1);
        FOR(i=0; i<=tmp_loop; i++)
        {

            tmp.a_fx[i] = sub(shr(X1.a_fx[i],sft),X2.a_fx[i]);
            move16();
            tmp.b_fx[i] = sub(shr(X1.b_fx[i],sft),X2.b_fx[i]);
            move16();
        }
        tmp.Q = X2.Q;
        move16();
    }
    ELSE
    {
        tmp_loop = shr(X1.lag_fx,1);
        FOR(i=0; i<=tmp_loop; i++)
        {

            tmp.a_fx[i] = sub(X1.a_fx[i],shl(X2.a_fx[i],sft));
            move16();
            tmp.b_fx[i] = sub(X1.b_fx[i],shl(X2.b_fx[i],sft));
            move16();
        }
        tmp.Q = X1.Q;
        move16();
    }
    tmp.lag_fx = s_max(X1.lag_fx, X2.lag_fx) ;
    tmp.nH_fx=s_max(X1.nH_fx,X2.nH_fx);
    tmp.nH_4kHz_fx=s_max(X1.nH_4kHz_fx,X2.nH_4kHz_fx);
    tmp.upper_cut_off_freq_of_interest_fx=X1.upper_cut_off_freq_of_interest_fx;
    move16();
    tmp.upper_cut_off_freq_fx=X1.upper_cut_off_freq_fx;
    move16();
    return tmp ;
}

void DTFS_fast_fs_inv_fx( DTFS_STRUCTURE_FX *X_fx,Word16 *out_fx, Word16 N_fx, Word16 LOG2N)
{
    Word16 i, M_2, N_2, s;
    Word16 dbuf_fx[256+1];

    M_2=s_min(shr(X_fx->lag_fx,1),X_fx->nH_fx);
    move16();
    N_2=shr(N_fx,1);
    s=negate(X_fx->Q);

    dbuf_fx[0]=X_fx->a_fx[0];
    move16();
    dbuf_fx[1]=0;
    move16();

    FOR (i=1; i<=M_2; i++)
    {
        dbuf_fx[2*i]=(Word16)shift_r(X_fx->a_fx[i],s);
        move16();
        dbuf_fx[2*i+1]=(Word16)shift_r(X_fx->b_fx[i],s);
        move16();
    }

    FOR ( ; i<N_2; i++)
    {
        dbuf_fx[2*i]=0;
        move16();
        dbuf_fx[2*i+1]=0;
        move16();
    }

    /*  do IFFT */
    r_fft_4_fx(dbuf_fx, N_fx, sub(LOG2N,1), -1);

    FOR (i=0; i<N_fx; i++)
    {
        out_fx[i]=dbuf_fx[i];/* Q0 */         move16();
    }
}
/*==============================================================================*/
/* FUNCTION      :  DTFS_freq_corr_fx ()                                        */
/*------------------------------------------------------------------------------*/
/* PURPOSE       :                                                              */
/*------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                           */
/*   _ (struct DTFS_STRUCTURE_FX) X1_DTFS_fx :  a_fx/b_fx in X1_DTFS_fx.Q       */
/*   _ (struct DTFS_STRUCTURE_FX) X2_DTFS_fx :  a_fx/b_fx in X2_DTFS_fx.Q       */
/*   _ (Word16) lband:  Q0                                                      */
/*   _ (Word16) hband:  Q0                                                      */
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                           */
/*   _ (Word16) *Qout :        Q of output result                               */
/*------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                     */
/*                    _ None                                                    */
/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                           */
/*   _ (Word32) Result :       Qout                                             */
/*------------------------------------------------------------------------------*/
/* CALLED FROM : TX                                                             */
/*==============================================================================*/

Word32 DTFS_freq_corr_fx( DTFS_STRUCTURE_FX X1_DTFS_fx, DTFS_STRUCTURE_FX X2_DTFS_fx, Word16 lband, Word16 hband, Word16 *Qout)
{
    Word16 k, HalfLag, lk, hk ;
    Word32 corr_fx;
    Word32 freq_fx, L_lband, L_hband;
    Word32 E_fx;
    Word32 Num, Den, Result;
    Word16 E1_fx, E2_fx, Q1, Q2, Qr;
    Word16 expa,expb,fraca,fracb,scale;
    Word16 exp,tmp;
    Word32 L_tmp;
    Word16 Q_num,Q_den;

    IF (sub(X1_DTFS_fx.lag_fx ,X2_DTFS_fx.lag_fx)< 0)
    {
        DTFS_zeroPadd_fx(X2_DTFS_fx.lag_fx,&X1_DTFS_fx) ;
    }

    corr_fx = L_deposit_l(0);

    L_lband = L_mult(lband, X2_DTFS_fx.lag_fx);
    L_hband = L_mult(hband, X2_DTFS_fx.lag_fx);
    HalfLag = s_min(shr(X2_DTFS_fx.lag_fx, 1),X2_DTFS_fx.nH_4kHz_fx);

    /* get lband and hband */
    FOR (k=0; k<=HalfLag; k++)
    {
        freq_fx = L_mult(k, 12800);
        IF (L_sub(freq_fx, L_lband)>=0)
        {
            BREAK;
        }
    }
    lk = k;
    FOR (k=0; k<=HalfLag; k++)
    {
        freq_fx = L_mult(k, 12800);
        IF (L_sub(freq_fx, L_hband)>=0)
        {
            BREAK;
        }
    }
    hk = k;
    move16();

    FOR (k=lk; k<hk; k++)
    {
        corr_fx = L_mac0(corr_fx, X1_DTFS_fx.a_fx[k], X2_DTFS_fx.a_fx[k]);       /*  Q(1) */
        corr_fx = L_mac0(corr_fx, X1_DTFS_fx.b_fx[k], X2_DTFS_fx.b_fx[k]);       /*  Q(1) */
    }

    Qr = norm_l(corr_fx);
    if (corr_fx==0)
    {
        Qr = 31;
        move16();
    }

    E1_fx = round_fx(L_shl(corr_fx, Qr));      /*  Q(Qr-16) */
    Num = L_mult0(E1_fx, E1_fx);                            /*  Q(2+2*Qr-32+1) */
    Q_num = sub(shl(add(add(X1_DTFS_fx.Q,X2_DTFS_fx.Q),Qr),1),32);

    /*  PORTING: Handling the functions with variable no. of arguments */
    E_fx = DTFS_getEngy_band_fx(X1_DTFS_fx,lband, hband);                    /*  Q(1) */
    Q1 = norm_l(E_fx);
    if (E_fx==0)
    {
        Q1 = 31;
        move16();
    }

    E1_fx = round_fx(L_shl(E_fx, Q1));         /*  Q(1+Q1-16) */

    /*  PORTING: Handling the functions with variable no. of arguments */
    E_fx = DTFS_getEngy_band_fx(X2_DTFS_fx,lband, hband);                    /*  Q(1) */
    Q2 = norm_l(E_fx);
    if (E_fx==0)
    {
        Q2 = 31;
        move16();
    }

    E2_fx = round_fx(L_shl(E_fx, Q2));         /*  Q(1+Q2-16) */

    Den = L_mult0(E1_fx, E2_fx);                            /*  Q(2+Q1+Q2-32+1) */
    Q_den = sub(add(shl(add(X2_DTFS_fx.Q,X1_DTFS_fx.Q),1),add(Q1,Q2)),32);

    Num = L_max(Num, 1);

    expa = norm_l(Num);
    fraca = extract_h(L_shl(Num,expa));
    expa = sub(30, add(expa,Q_num));


    expb = norm_l(Den);
    fracb = round_fx(L_shl(Den,expb));
    expb = sub(30, add(expb,Q_den));


    scale = shr(sub(fraca,fracb),15);
    fracb = shl(fracb,scale);
    expb = sub(expb,scale);

    tmp = div_s(fracb,fraca);
    exp = sub(expb,expa);

    L_tmp = Isqrt_lc(L_deposit_h(tmp),&exp); /* Q(31-exp) */
    IF (corr_fx>0)
    {
        Result = L_add(L_tmp, 0);
    }
    ELSE
    {
        Result = L_negate(L_tmp);
    }
    *Qout = sub(30,exp);

    return Result;
}

/*===================================================================*/
/* FUNCTION      :  DTFS_alignment_weight_fx ()                      */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  search for alignment                             */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) X1_fx :  a/b in X1.Q               */
/*   _ (struct DTFS_STRUCTURE_FX) X2_fx :  a/b in X2.Q               */
/*   _ (Word16) Eshift: Q7                                           */
/*   _ (Word16 *) LPC1:  lpc coefficients. Q12                       */
/*   _ (Word16 *) LPC2:  lpc coefficients. Q12                       */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16) fshift_fx :           Q7                             */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

Word16 DTFS_alignment_weight_fx( DTFS_STRUCTURE_FX *X_fx, DTFS_STRUCTURE_FX X2, Word16 Eshift, const Word16 *LPC1, const Word16 *LPC2, Word16 *S_fx, Word16 *C_fx, Word16 *pf_temp1,Word16 *pf_temp2, Word16 *pf_temp, Word16 *pf_n2)
{
    /* Eshift is w.r.t  X2  */
    Word16 k, HalfLag, start, end, ab1[MAXLAG_WI], ab2[MAXLAG_WI] ;
    Word32 corr_fx;
    Word32 maxcorr_fx, wcorr_fx, diff_corr;
    DTFS_STRUCTURE_FX X1 ;
    Word16 temp, temp1, Qcorr, Qmaxcorr, inv_lag, n_fx, diff_fx, fshift_fx, Adiff_fx;
    Word16 tmplpc_fx[M+1];
    Word16 exp,tmp;
    Word32 L_tmp;

    diff_fx = 0;        /* to avoid compilation warnings */


    DTFS_copy_fx(&X1,*X_fx);/* X1 = *X_fx ; */
    DTFS_adjustLag_fx(&X1,X2.lag_fx) ;

    /*  PORTING: Handling the functions with variable no. of arguments */
    poleFilter_setup_fx(LPC1, add(M,1), X1,S_fx,C_fx,pf_temp1,pf_temp2,pf_temp,pf_n2);
    DTFS_poleFilter_fx_9(&X1, pf_temp1, pf_temp2, pf_temp, pf_n2);
    FOR (k=0; k<M+1; k++)
    {
        tmplpc_fx[k]=mult_r(LPC1[k], pwf_fx[k]);          /*  Q12 */
    }
    DTFS_zeroFilter_fx(&X1,tmplpc_fx,M+1, S_fx, C_fx) ;

    /*  PORTING: Handling the functions with variable no. of arguments */
    poleFilter_setup_fx(LPC2, add(M,1),X2,S_fx,C_fx,pf_temp1,pf_temp2,pf_temp,pf_n2);
    DTFS_poleFilter_fx_9(&X2, pf_temp1, pf_temp2, pf_temp, pf_n2);
    FOR (k=0; k<M+1; k++)
    {
        tmplpc_fx[k]=mult_r(LPC2[k], pwf_fx[k]);          /*  Q12 */
    }
    DTFS_zeroFilter_fx(&X2,tmplpc_fx,M+1, S_fx, C_fx) ;

    maxcorr_fx = L_add(MIN_32, 0);
    Qmaxcorr = 0;
    move16();
    fshift_fx = Eshift ;
    move16();
    Adiff_fx = (Word16)s_max(768, mult_r(4915, shl(X2.lag_fx, 7)));    /*  Q7, 768=6*128, 4915 = 0.15*32768 */

    if (sub(X2.lag_fx,60) < 0)
    {
        diff_fx = 32;
        move16(); /*  Q7 of 0.25 */
    }
    if (sub(X2.lag_fx,60) >= 0)
    {
        diff_fx = 64;
        move16(); /*  Q7 of 0.5 */
    }

    HalfLag = s_min(shr(X2.lag_fx, 1),X2.nH_4kHz_fx);
    exp = norm_s(X2.lag_fx);
    tmp =  div_s(shl(1,sub(14,exp)),X2.lag_fx);
    L_tmp = L_shl(tmp,exp+6);
    inv_lag = round_fx(L_tmp);

    FOR (k=0; k<=HalfLag; k++)
    {
        ab1[k] = round_fx(L_mac0(L_mult0(X1.a_fx[k], X2.a_fx[k]), X1.b_fx[k], X2.b_fx[k])); /*  Q(-15) */
        ab2[k] = round_fx(L_msu0(L_mult0(X1.a_fx[k], X2.b_fx[k]), X1.b_fx[k], X2.a_fx[k])); /*  Q(-15) */
    }

    start=sub(Eshift, Adiff_fx);
    end=add(Eshift, Adiff_fx);

    FOR (n_fx=start; n_fx<=end; n_fx+=diff_fx)
    {
        /*  Q7 */
        corr_fx = L_deposit_l(0);
        temp = 0;
        move16();
        temp1 = round_fx(L_shr(L_mult(inv_lag, n_fx), 2));       /*  Q(19+7+1-2-16)=Q9 of n/X2.lag */

        if (temp1<0)
        {
            temp1=add(temp1, 0x200);                       /*  avoid negative, 0x200 is Q9 of 1 ( 2*pi ) */
        }

        FOR (k=0; k<=HalfLag; k++)
        {
            corr_fx = L_mac0(corr_fx, ab1[k], cos_table[s_and(temp,511)]);
            corr_fx = L_mac0(corr_fx, ab2[k], cos_table[s_and((temp+128),511)]);
            temp = add(temp, temp1);
        }
        temp = sub(8192, mult_r(20972, abs_s(sub(n_fx, Eshift))));  /*  Q13, 20972 = Q21 of 0.01. */
        Qcorr = norm_l(corr_fx);
        if(corr_fx==0)
        {
            Qcorr = 31;
            move16();
        }

        temp1 = round_fx((Word32)L_shl(corr_fx, Qcorr));        /*  Q(Qcorr-16) */
        wcorr_fx = L_mult(temp1, shl(temp, 2));                     /*  Q(Qcorr-16+13+2+1)=Q(Qcorr) */

        IF (sub(Qmaxcorr,Qcorr) >= 0)
        {
            diff_corr = L_sub(wcorr_fx, L_shl(maxcorr_fx, sub(Qcorr, Qmaxcorr)));    /*  Qcorr */
        }
        ELSE
        {
            diff_corr = L_sub(L_shl(wcorr_fx, sub(Qmaxcorr, Qcorr)), maxcorr_fx);   /*  Qmaxcorr */
        }

        if ( diff_corr > 0 )
        {
            fshift_fx = n_fx ;
            move16();
            maxcorr_fx = L_add(wcorr_fx, 0);
            Qmaxcorr = Qcorr;
            move16();
        }
    }

    return fshift_fx ;
}
/*===================================================================*/
/* FUNCTION      :  DTFS_alignment_extract_td_fx ()                  */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  search for alignment in time domain              */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16 *) x1: Q?                                             */
/*   _ (Word16 *) x2: Q?                                             */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16) lag :           Q0                                   */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ (Word16 *) idx:  Q0                          */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX                                                  */
/*===================================================================*/

Word16 DTFS_alignment_extract_td_fx(Word16 *x1, Word16 *x2, Word16 lag)
{
    Word16 j, k, idx, Adiff_fx ;
    Word32 maxcorr, corr;

    maxcorr = L_add(MIN_32, 0);
    Adiff_fx = (Word16)(s_max(4, shr(lag, 3)));

    idx=0;
    move16();
    FOR ( j=negate(Adiff_fx); j<=Adiff_fx; j++)
    {
        corr=L_deposit_l(0);
        FOR (k=0; k<lag; k++)
        {
            corr=L_mac(corr,x1[k],x2[(k-j+lag)%lag]);
        }
        if (L_sub(corr,maxcorr)>0)
        {
            idx = j ;
            move16();
            maxcorr = L_add(corr, 0);
        }
    }
    return idx;
}
/*===================================================================*/
/* FUNCTION      :    DTFS_alignment_fine_new_fx ()                  */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  search for alignment                             */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_fx) X1_fx :  a/b in X1_fx.Q                      */
/*   _ (struct DTFS_fx) X2_fx :  a/b in X2_fx.Q                      */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16) fshift_fx :           Q2                             */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX                                                  */
/*===================================================================*/

Word16 DTFS_alignment_fine_new_fx( DTFS_STRUCTURE_FX X1_fx, DTFS_STRUCTURE_FX X2_fx, Word16 *S_fx, Word16 *C_fx)
{
    Word16 temp, temp1, k, Qcorr, Qmaxcorr;
    Word16 n, fshift_fx, HalfLag, ab1[MAXLAG_WI], ab2[MAXLAG_WI];
    Word32 corr_fx;
    Word32 maxcorr_fx, wcorr_fx, diff_corr;

    IF (sub(X1_fx.lag_fx, X2_fx.lag_fx) < 0)
    {
        DTFS_zeroPadd_fx(X2_fx.lag_fx, &X1_fx) ;
    }

    maxcorr_fx = L_add(MIN_32, 0);
    Qmaxcorr = 0;
    move16();
    HalfLag = s_min(shr(X2_fx.lag_fx, 1),X2_fx.nH_fx);

    FOR (k=0; k<=HalfLag; k++)
    {
        ab1[k] = round_fx(L_mac(L_mult(X1_fx.a_fx[k],X2_fx.a_fx[k]),X1_fx.b_fx[k],X2_fx.b_fx[k]));
        ab2[k] = round_fx(L_msu(L_mult(X1_fx.b_fx[k],X2_fx.a_fx[k]),X1_fx.a_fx[k],X2_fx.b_fx[k]));
    }


    fshift_fx = 0;
    move16();
    FOR ( n=-76; n<=80; n+=4 )
    {
        /*  n is Q2 */
        corr_fx = L_deposit_l(0);
        temp = 0;
        move16();
        temp1 = n;
        move16();

        IF (n<0)
        {
            temp1=add(temp1, shl(X2_fx.lag_fx, 2));           /*  avoid negative */
        }

        FOR (k=0; k<=HalfLag; k++)
        {
            corr_fx=L_mac(corr_fx, ab1[k], C_fx[temp%(4*X2_fx.lag_fx)]);
            corr_fx=L_mac(corr_fx, ab2[k], S_fx[temp%(4*X2_fx.lag_fx)]);
            temp = add(temp, temp1);
        }
        temp=sub(32767, extract_l(L_shr(L_mult(82, abs_s(n)), 1)));    /*  Q15 */
        Qcorr = norm_l(corr_fx);
        if(corr_fx==0)
        {
            Qcorr = 31;
            move16();
        }

        temp1 = round_fx((Word32)L_shl(corr_fx, Qcorr));        /*  Q(Qcorr-16) */
        wcorr_fx = L_mult(temp1, temp);                     /*  Q(Qcorr-16+15+1)=Q(Qcorr) */

        IF (sub(Qmaxcorr,Qcorr) >= 0)
        {
            diff_corr = L_sub(wcorr_fx, L_shl(maxcorr_fx, sub(Qcorr, Qmaxcorr)));    /*  Qcorr */
        }
        ELSE
        {
            diff_corr = L_sub(L_shl(wcorr_fx, sub(Qmaxcorr, Qcorr)), maxcorr_fx);   /*  Qmaxcorr */
        }

        if ( diff_corr > 0 )
        {
            fshift_fx = n ;
            move16();
            maxcorr_fx = (Word32)L_shl(corr_fx, Qcorr) ;                /*  Qcorr */
            Qmaxcorr = Qcorr;
            move16();
        }
    }

    return fshift_fx ;
}
/*===========================================================================*/
/* FUNCTION      :  DTFS_alignment_full_fx ()                                     */
/*---------------------------------------------------------------------------*/
/* PURPOSE       :  search for alignment                                     */
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                        */
/*   _ (struct DTFS_STRUCTURE_FX) X1_DTFS_fx :  a_fx/b_fx in X1_DTFS_fx.Q    */
/*   _ (struct DTFS_STRUCTURE_FX) X2_DTFS_fx :  a_fx/b_fx in X2_DTFS_fx.Q    */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                        */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                        */
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                        */
/*   _ (Word16) fshift_fx :           Q1                                     */
/*---------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                  */
/*                    _ None                                                 */
/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                                */
/*---------------------------------------------------------------------------*/
/* CALLED FROM : TX                                                          */
/*===========================================================================*/

Word16 DTFS_alignment_full_fx( DTFS_STRUCTURE_FX X1_DTFS_fx, DTFS_STRUCTURE_FX X2_DTFS_fx, Word16 ph_offset_fx,Word16 *S_fx, Word16 *C_fx
                               , Word16 FR_flag
                             )
{
    Word16 temp, temp1, k, start, end, HalfLag, ab1[MAXLAG_WI], ab2[MAXLAG_WI] ;
    Word16 n, fshift_fx;
    Word32 corr_fx, maxcorr_fx;
    Word16 Eshift, Adiff_fx;


    /* Calculating the expected alignment shift */
    Eshift = mult_r(ph_offset_fx, shl(X2_DTFS_fx.lag_fx, 7));      /* confirmed I<2 by smv12.org, Q7 */
    find_rem((Word16)L_FRAME, shr(add(X2_DTFS_fx.lag_fx, X1_DTFS_fx.lag_fx), 1), &temp);
    temp = add(shl(temp, 7), Eshift);                            /*  Q7 */

    IF (temp<0)
    {
        temp = add(temp, shl(X1_DTFS_fx.lag_fx, 7));                /*  Q7 */
    }
    find_rem(temp, shl(X1_DTFS_fx.lag_fx, 7), &Eshift);                   /*  Q7 */
    Eshift=shl(shr(Eshift,7),1); /* Q1 but integer */

    IF (sub(X2_DTFS_fx.lag_fx,60) > 0)
    {
        Adiff_fx = shl(X2_DTFS_fx.lag_fx,1-3); /* lag_fx/8 in Q1 */
    }
    ELSE
    {
        Adiff_fx = shl(X2_DTFS_fx.lag_fx,1-2); /* lag_fx/4 in Q1 */
    }


    IF (X1_DTFS_fx.lag_fx < X2_DTFS_fx.lag_fx)
    {
        DTFS_zeroPadd_fx(X2_DTFS_fx.lag_fx,&X1_DTFS_fx) ;
    }

    maxcorr_fx = L_add(MIN_32, 0);
    HalfLag = s_min(shr(X2_DTFS_fx.lag_fx, 1),X2_DTFS_fx.nH_4kHz_fx);

    FOR (k=0; k<=HalfLag; k++)
    {
        ab1[k] = round_fx(L_mac(L_mult(X1_DTFS_fx.a_fx[k], X2_DTFS_fx.a_fx[k]), X1_DTFS_fx.b_fx[k], X2_DTFS_fx.b_fx[k]));   /*  Q(-15); */
        ab2[k] = round_fx(L_msu(L_mult(X1_DTFS_fx.b_fx[k], X2_DTFS_fx.a_fx[k]), X1_DTFS_fx.a_fx[k], X2_DTFS_fx.b_fx[k]));   /*  Q(-15); */
    }
    IF (FR_flag == 0)
    {
        start=sub(Eshift, Adiff_fx); /*Q1*/
        end=add(Eshift, Adiff_fx);   /*Q1*/
    }
    ELSE
    {
        /*
        in FR mode, we cannot save and cary forward ph_offset as in VBR mode encoder "ph_offset_E_fx",
        so we set ph_offset_fx = 0, as a result we cannot accurately estimate the expected alignment shift Eshift to limit the search,
        so we search the full range [0 to X2_DTFS_fx.lag_fx] similar to FL
        */
        start=0; /*Q1*/
        end=shl(X2_DTFS_fx.lag_fx,1);   /*Q1*/
    }

    fshift_fx=start;
    move16();   /*Q1*/

    FOR (n=start; n<=end; n++) /*n in Q1*/
    {
        /*  Q1 */
        corr_fx = L_deposit_l(0);
        temp = 0;
        move16();
        temp1 = add(n,shl(X2_DTFS_fx.lag_fx,1)); /* add lag_fx in Q1to make positive */
        FOR (k=0; k<=HalfLag; k++)
        {
            corr_fx = L_mac(corr_fx, ab1[k], C_fx[(2*temp)%(4*X2_DTFS_fx.lag_fx)]);
            corr_fx = L_mac(corr_fx, ab2[k], S_fx[(2*temp)%(4*X2_DTFS_fx.lag_fx)]);
            temp = add(temp, temp1);
        }

        if (L_sub(corr_fx, maxcorr_fx) > 0)
        {
            fshift_fx = n ;   /*  Q1 */ move16();
            maxcorr_fx = L_add(corr_fx, 0);
        }
    }

    return fshift_fx ; /*Q1*/
}
/*===================================================================*/
/* FUNCTION      :  DTFS_phaseShift_fx ()                                 */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Phase is shifted by 2pi*ph/Lag                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) ph:    shift index, Q7                               */
/*   _ (Word16) Lag:   Pitch Lag value as for shift                  */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ None.                                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (struct DTFS_fx) X1 :  a/b in X1.Q                                   */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void DTFS_phaseShift_fx( DTFS_STRUCTURE_FX *X_fx,Word16 ph, Word16 Lag, Word16 *S_fx, Word16 *C_fx)
{
    Word16 k ;
    Word16 temp, HalfLag;
    Word32 L_temp, L_temp2;

    L_temp2 = L_deposit_l(0);
    HalfLag = s_min(shr(X_fx->lag_fx, 1),X_fx->nH_fx);

    IF (ph>0)
    {
        FOR (k=0; k<=HalfLag; k++)
        {
            temp = X_fx->a_fx[k];
            L_temp = L_shr(L_temp2, 5);   /*  Q2 */
            X_fx->a_fx[k] = round_fx(L_msu(L_mult(temp, C_fx[L_temp%(4*Lag)]), X_fx->b_fx[k], S_fx[L_temp%(4*Lag)])) ;  /*  X.Q */
            X_fx->b_fx[k] = round_fx(L_mac(L_mult(X_fx->b_fx[k], C_fx[L_temp%(4*Lag)]), temp, S_fx[L_temp%(4*Lag)])) ;
            L_temp2 = L_add(L_temp2, ph);
        }
    }

    IF (ph<0)
    {
        FOR (k=0; k<=HalfLag; k++)
        {
            temp = X_fx->a_fx[k];
            L_temp = L_shr(L_negate(L_temp2), 5);   /*  Q2 */
            X_fx->a_fx[k] = round_fx(L_mac(L_mult(temp, C_fx[L_temp%(4*Lag)]), X_fx->b_fx[k], S_fx[L_temp%(4*Lag)])) ;   /*  X.Q */
            X_fx->b_fx[k] = round_fx(L_msu(L_mult(X_fx->b_fx[k], C_fx[L_temp%(4*Lag)]), temp, S_fx[L_temp%(4*Lag)])) ;
            L_temp2 = L_add(L_temp2, ph);
        }
    }
}

/*===================================================================*/
/* FUNCTION      :  Q2phaseShift_fx ()                               */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Phase is shifted by 2pi*ph/Lag                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) ph:    shift index, Q2                               */
/*   _ (Word16) Lag:   Pitch Lag value as for shift                  */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ None.                                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx :  a/b in X_fx->Q             */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void Q2phaseShift_fx( DTFS_STRUCTURE_FX *X_fx,Word16 ph, Word16 Lag, Word16 *S_fx, Word16 *C_fx)
{
    Word16 k ;
    Word16 temp,  HalfLag;
    Word32 temp2;

    temp2 = L_deposit_l(0);

    HalfLag = s_min(shr(X_fx->lag_fx, 1),X_fx->nH_fx);


    IF (ph>0)
    {
        FOR (k=0; k<=HalfLag; k++)
        {
            temp = X_fx->a_fx[k];
            X_fx->a_fx[k] = round_fx(L_msu(L_mult(temp, C_fx[temp2%(4*Lag)]), X_fx->b_fx[k], S_fx[temp2%(4*Lag)])) ; /*  X.Q */
            X_fx->b_fx[k] = round_fx(L_mac(L_mult(X_fx->b_fx[k], C_fx[temp2%(4*Lag)]), temp, S_fx[temp2%(4*Lag)])) ;
            temp2 = L_add(temp2, (Word32)ph);
        }
    }



    IF (ph<0)
    {
        FOR (k=0; k<=HalfLag; k++)
        {
            temp = X_fx->a_fx[k];
            X_fx->a_fx[k] = round_fx(L_mac(L_mult(temp, C_fx[temp2%(4*Lag)]), X_fx->b_fx[k], S_fx[temp2%(4*Lag)])) ;    /*  X.Q */
            X_fx->b_fx[k] = round_fx(L_msu(L_mult(X_fx->b_fx[k], C_fx[temp2%(4*Lag)]), temp, S_fx[temp2%(4*Lag)])) ;
            temp2 = add((Word16)temp2, negate(ph));
        }
    }
}
/*===================================================================*/
/* FUNCTION      :  DTFS_zeroPadd_fx ()                              */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  zeroPadding                                      */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) N_fx:    length  , Q0                                   */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ None.                                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx :  a/b in X_fx.Q i.e Q6       */
/*   _ (Word16) lag: pitch lag of *X_fx, Q0                          */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void DTFS_zeroPadd_fx(Word16 N_fx,DTFS_STRUCTURE_FX *X_fx)
{
    Word16 i, start, end ,diff_fx,rem_fx;

    if (sub(N_fx,X_fx->lag_fx) == 0)
    {
        return ;
    }
    start = add(shr(X_fx->lag_fx,1), 1);
    end = shr(N_fx, 1);

    move16();
    FOR (i=start; i<=end; i++)
    {
        X_fx->a_fx[i]=0;
        move16();
        X_fx->b_fx[i]=0;
        move16();
    }

    X_fx->lag_fx = N_fx ;
    move16();

    /* recompute nH for new lag */
    diff_fx = find_rem(12800,X_fx->lag_fx,&rem_fx);
    X_fx->nH_fx = find_rem(X_fx->upper_cut_off_freq_fx,diff_fx,&rem_fx);

    if(sub(sub(X_fx->upper_cut_off_freq_fx,shr((Word16)L_mult(diff_fx,X_fx->nH_fx),1)),diff_fx)>=0)
    {
        X_fx->nH_fx = add(X_fx->nH_fx,1);
    }
}
/*===================================================================*/
/* FUNCTION      :  to_fs_fx ()                                      */
/*-------------------------------------------------------------------*/
/* PURPOSE       :                                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16 *) x:    input time domain series. Q0                 */
/*   _ (Word16) N:    Lag                                            */
/*   _ (struct DTFS_STRUCTURE_FX) PREVCW_FX: a/b in PREVCW_FX.Q      */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) (Word16 a[]) :  in Q               */
/*   _ (struct DTFS_STRUCTURE_FX) (Word16 b[]) :  in Q               */
/*   _ (struct DTFS_STRUCTURE_FX) (Word16 lag) :  Q0                 */
/*   _ (struct DTFS_STRUCTURE_FX) (Word16 Q  ) :  Q value of a/b     */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ None.                                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
void DTFS_to_fs_fx(
    const Word16 *x,            /* i : time domain signal               */
    Word16   N,                 /* i : Length of input vector           */
    DTFS_STRUCTURE_FX *X_fx,    /* o : DTFS structure with a, b, lag    */
    Word16 Fs,                  /* i : sampling rate                    */
    Word16 FR_flag,             /* i :  FR flag                         */
    Word16 *S_fx,
    Word16 *C_fx
)

{
    Word16 n,temp,temp_neg,inv_lag,sum,diff_fx;
    Word16 nH, k, nH_band, nH_4kHz;
    Word32 L_temp, Lx0;
    Word32 L_a, L_b,L_tmp;
    Word32 La[MAXLAG_WI], Lb[MAXLAG_WI], Labmax;
    Word16 exp,tmp;
    Word32 L_tmp1;


    IF (!FR_flag)
    {
        IF (sub(Fs,16000)==0)
        {
            X_fx->upper_cut_off_freq_of_interest_fx=4000;
            move16();
            X_fx->upper_cut_off_freq_fx=6400;
            move16();
            X_fx->Fs_fx=INT_FS_FX;
            move16();
        }
        ELSE IF (sub(Fs,8000)==0)
        {
            X_fx->upper_cut_off_freq_of_interest_fx=3300;
            move16();
            X_fx->upper_cut_off_freq_fx=4000;
            move16();
            X_fx->Fs_fx=INT_FS_FX;
            move16();
        }
    }
    ELSE
    {
        X_fx->upper_cut_off_freq_of_interest_fx=8000;
        move16();
        X_fx->upper_cut_off_freq_fx=8000;
        move16();
        X_fx->Fs_fx=16000;
        move16();
    }
    X_fx->lag_fx = N ;
    move16();

    exp = norm_s(X_fx->lag_fx);
    tmp = div_s(shl(1,sub(14,exp)),X_fx->lag_fx); /* Q29-exp */
    L_tmp1 = L_mult(12800,tmp); /* Q(30-exp) */
    diff_fx = extract_h(L_shl(L_tmp1,sub(exp,14))); /* Q0 */

    exp = norm_s(diff_fx);
    tmp = div_s(shl(1,sub(14,exp)),diff_fx); /* Q29-exp */
    L_tmp1 = L_mult(X_fx->upper_cut_off_freq_fx,tmp); /* Q(30-exp) */
    nH_band = extract_h(L_shl(L_tmp1,sub(exp,14))); /* Q0 */

    nH_4kHz = mult(10240,(X_fx->lag_fx)); /* 4000/12800 in Q15 */

    if(sub(sub(X_fx->upper_cut_off_freq_fx,shr((Word16)L_mult(diff_fx,nH_band),1)),diff_fx)>=0)
    {
        nH_band = add(nH_band,1);
    }

    if(sub(sub(4000,shr((Word16)L_mult(diff_fx,nH_4kHz),1)),diff_fx)>=0)
    {
        nH_4kHz = add(nH_4kHz,1);
    }

    /* Number of harmonics excluding the ones at 0 and at pi */
    nH = shr(sub(N,1),1);

    /* The DC component */
    X_fx->a_fx[0] = 0 ;
    move16();
    X_fx->b_fx[0] = 0 ;
    move16();

    exp = norm_s(N);
    tmp = div_s(shl(1,(sub(14,exp))),N);
    L_tmp = L_shl(tmp,add(exp,6));
    inv_lag = round_fx(L_tmp);

    Lx0 = L_deposit_h(x[0]);
    Labmax = L_deposit_l(0);
    FOR ( k=1; k<=nH; k++ )
    {
        L_a = L_shr(Lx0,1) ;                             /*  Q16 */

        L_b = L_deposit_l(0);
        sum = k;
        move16();
        temp = k;
        move16();
        FOR ( n=1; n<N; n++ )
        {
            L_a = L_mac0(L_a, x[n], C_fx[(4*sum)%(4*N)]);  /*  Q16 of x[n]*cos(sum) */
            L_b = L_mac0(L_b, x[n], S_fx[(4*sum)%(4*N)]);  /*  Q16 of x[n]*sin(sum) */
            sum =  add(sum,temp);
        }
        La[k]=L_shr(L_a, 6);          /*  Q8 of a[k]*2.0 */
        Lb[k]=L_shr(L_b, 6);          /*  Q8 of b[k]*2.0 */

        L_temp=L_abs(La[k]);

        if (L_sub(L_temp,Labmax)>0)
        {
            Labmax=L_temp;
        }
        L_temp=L_abs(Lb[k]);

        if (L_sub(L_temp,Labmax)>0)
        {
            Labmax=L_temp;
        }
    }


    /* The harmonic at 'pi' */

    /* IF ( N%2 == 0 )  */
    IF (s_and (N,1) == 0 )
    {

        L_a = L_deposit_l(0);
        temp = 1 ;
        move16();
        temp_neg = negate(temp);
        FOR ( n=0; n<N-1; n+=2 )
        {
            L_a = L_mac(L_a, x[n], temp);               /*  Q1 */
            L_a = L_mac(L_a, x[n+1], temp_neg);
            /*temp= negate(temp); */
        }
        if(s_and(N,1)) /*if N is odd we need to calculate last  */
        {
            L_a = L_mac(L_a, x[n], temp);               /*  Q1 */
        }

        La[k]=L_shl(L_a, 7);
        move32();       /*  Q8 of a[k]*1.0 */



        L_temp=L_abs(La[k]);

        if (L_sub(L_temp,Labmax)>0)
        {
            Labmax = L_add(L_temp, 0);
        }

        X_fx->b_fx[k] = 0 ;
        move16();
    }

    temp = norm_l(Labmax);
    if (Labmax==0)
    {
        temp = 31;
        move16();
    }

    FOR ( k=1; k<=nH; k++ )
    {
        X_fx->a_fx[k]=round_fx(L_shl(La[k], temp));                     /* Q(8+temp-16)=Q(temp-8) */
        X_fx->a_fx[k]=mult_r(X_fx->a_fx[k], inv_lag);
        move16();         /* Q(temp-8+19+1-16)=Q(temp-4) of a[k]*2.0/N */
        X_fx->b_fx[k]=round_fx(L_shl(Lb[k], temp));                     /* Q(8+temp-16)=Q(temp-8) */
        X_fx->b_fx[k]=mult_r(X_fx->b_fx[k], inv_lag);
        move16();        /* Q(temp-8+19+1-16)=Q(temp-4) of b[k]*2.0/N */
    }

    /* IF ( N%2 == 0 ) */
    IF ( s_and(N,1) == 0 )
    {
        X_fx->a_fx[k]=round_fx(L_shl(La[k], temp));                     /* Q(8+temp-16)=Q(temp-8) */
        X_fx->a_fx[k]=mult_r(X_fx->a_fx[k], inv_lag);
        move16();         /* Q(temp-8+19+1-16)=Q(temp-4) of a[k]*1.0/N */
        X_fx->b_fx[k]=0;
        move16();
    }

    X_fx->Q=sub(temp, 4);

    tmp = s_min(shr(X_fx->lag_fx,1),sub(MAXLAG_WI,1));
    FOR(k=add(nH_band,1); k<=tmp; k++)
    {
        X_fx->a_fx[k]=0;
        move16();
        X_fx->b_fx[k]=0;
        move16();
    }
    X_fx->nH_fx=nH_band;
    move16();
    X_fx->nH_4kHz_fx=nH_4kHz;
    move16();

}


/*===================================================================*/
/* FUNCTION      :  DTFS_transform_fx ()                             */
/*-------------------------------------------------------------------*/
/* PURPOSE       :                                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx:     a/b in X1.Q, lag in Q0   */
/*   _ (struct DTFS_STRUCTURE_FX) X2_fx. a/b in X2.Q, lag in Q0      */
/*   _ (Word16 *) phase_fx: 2pi normalized, Q12                      */
/*   _ (Word16) N:      length of series.                            */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 *) out_fx: output transformed series. Q0.             */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ None.                                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
void DTFS_transform_fx(
    DTFS_STRUCTURE_FX X_fx,         /* i : Starting DTFS to use in WI    */
    DTFS_STRUCTURE_FX X2_fx,        /* i : Ending DTFS to use in WI      */
    const Word32 *phase_fx,         /* i : Phase contour                 */
    Word16 *out_fx,                 /* o : Output time domain waveform   */
    Word16   N,                     /* i : Number of samples to generate */
    Word16 FR_flag                  /* i : Flag to indicate called in FR context */
)
{

    Word16 i, j ;
    Word32 L_temp_fx;
    Word16 w_fx;
    Word16 inv1_fx, inv2_fx, inv_fx, q1, q2;
    Word32 Lw_fx,temp32_fx;
    Word16 x2_256_fx[256], x1_256_fx[256];
    Word16 k, m, l1;
    Word16 tmptmp1_fx, tmptmp2_fx;
    Word32 tmptmp1_40_fx, tmptmp2_40_fx;
    Word16 N1;
    Word16 nrg_flag = 0;
    Word32 L_tmp;
    Word16 tmp, tmp1, tmp2, frac, exp1, exp2;
    Word16 expa, expb, fraca, fracb, scale;

    DTFS_STRUCTURE_FX *tmp2_dtfs_fx = DTFS_new_fx();
    DTFS_STRUCTURE_FX *tmp1_dtfs_fx = DTFS_new_fx();
    DTFS_STRUCTURE_FX *tmp3_dtfs_fx = DTFS_new_fx();
    DTFS_copy_fx(tmp1_dtfs_fx,X_fx);
    DTFS_copy_fx(tmp2_dtfs_fx,X2_fx);

    tmp2 = 0;   /* to avoid compilation warnings */


    DTFS_fast_fs_inv_fx(tmp1_dtfs_fx,x1_256_fx,256,8);
    DTFS_fast_fs_inv_fx(tmp2_dtfs_fx,x2_256_fx,256,8);



    /* L_temp_fx=invert_dp((Word40)N,4,&n,1);         = 1/M, Q(61-n) */
    /*    inv1_fx=round_fx(L_temp_fx);          = 1/M in Q(45-n) */
    /*    q1=sub(n,15); */
    exp1 = norm_s(N);
    inv1_fx = div_s(shl(1,sub(14,exp1)),N);    /* 29-exp1 */
    q1=add(exp1,1);

    tmp1 = sub(X_fx.lag_fx,N);
    exp2 = norm_s(tmp1);

    if(tmp1<0)
    {
        tmp2 = negate(tmp1);
    }
    tmp = div_s(shl(1,(14- exp2)),tmp2);    /* 29-exp2 */
    L_tmp = L_shl(tmp,16);

    if(tmp1<0)
    {
        L_tmp = L_negate(L_tmp);
    }

    q2=sub(exp2,3);



    /*L_temp_fx=(Word32) Mpy_32_16(extract_h(L_tmp),extract_l(L_tmp),22904); move32(); */ /*  L_temp=log(0.2)*log10(e)/(lag-M), Q(61-n) */
    L_temp_fx= Mult_32_16(L_tmp,22904); /*  L_temp=log(0.2)*log10(e)/(lag-M), Q(61-n) */
    inv2_fx=round_fx(L_temp_fx);          /*  log(0.2)*log10(e)/(lag-M), Q(45-n) */
    /* q2=sub(n,19);  */                   /* adjust Q factor to Q26 */


    IF (sub(sub(N, WI_SAMPLE_THLD), X_fx.lag_fx) > 0 )
    {
        inv_fx = inv2_fx;
        move16();
    }
    ELSE
    {
        inv_fx = inv1_fx;
        move16();
        exp2 = exp1;
        move16();
    }

    Lw_fx = L_deposit_l(inv_fx);

    FOR (i=0; i<N; i++)
    {
        IF (FR_flag==0)
        {
            IF ( sub(sub(N, WI_SAMPLE_THLD), X_fx.lag_fx) > 0 )
            {

                L_tmp = L_shl(Lw_fx,q2);            /* Q29-exp2+q2     */
                L_tmp = Mult_32_16(L_tmp, 27213); /* 3.321928 in Q13 -> 16+13+1 */ /*27-exp2 */
                L_tmp = L_shl(L_tmp, sub(exp2,add(q2,11)));
                frac = L_Extract_lc(L_tmp, &exp1); /* Extract exponent  */
                L_temp_fx =Pow2(14, frac);
                exp1 = sub(exp1,14);
                L_temp_fx = L_shl(L_temp_fx,add(exp1,15)); /* Q15 */

                w_fx=sub(16384,extract_h(L_shl(L_temp_fx,15))); /*  w_fx in Q14 1- exp(- (i+1) * log(.2)/(lag-M)) */
            }
            ELSE
            {
                w_fx=round_fx(L_shl(Lw_fx,q1));    /*  Q14 */
            }
        }
        ELSE
        {
            IF (nrg_flag)
            {
                w_fx=round_fx(L_shl(Lw_fx,q1));    /*  Q14 */
            }
            ELSE
            {

                N1 = sub(N , tmp2_dtfs_fx->lag_fx);

                IF (sub(i,N1) < 0)
                /*   w =  (i+1)/N1;  */
                {

                    /* w =  (i+1)/N1; */
                    IF(N1)
                    {
                        expa = norm_s(N1);
                        fraca = shl(N1,expa);
                        expa = sub(14,expa);

                        tmp = add(i,1);
                        expb = norm_s(tmp);
                        fracb = shl(tmp,expb);
                        expb =  sub(14,expb);

                        scale = shr(sub(fraca,fracb),15);
                        fracb = shl(fracb,scale);
                        expb = sub(expb,scale);

                        w_fx = div_s(fracb,fraca);
                        exp1 = sub(expb,expa);
                        w_fx = shl(w_fx,exp1-1);  /*Q14*/
                    }
                    ELSE
                    {
                        w_fx = 0;
                    }

                    Lw_fx=L_deposit_h(w_fx);


                }
                ELSE
                {
                    w_fx = 16384;
                    move16();
                }
            }
        }

        Lw_fx=L_add(Lw_fx,inv_fx);  /*  (i+1)*inv */
        /* mapping phase to 8x256 length signal */
        temp32_fx = phase_fx[i];  /* Q(27-11)=Q16 due to multiplication by pow(2.0,11) */
        j = rint_new_fx(temp32_fx);
        j = s_and(j,0x07ff);

        /*   k=j%8; */
        k=s_and(j,7);
        l1=shr(j,3);         /* reminder and quotient */

        tmptmp1_40_fx = L_deposit_l(0);
        tmptmp2_40_fx = L_deposit_l(0);
        FOR (j=0; j<12; j++)
        {
            m=(1000*LL+l1-OSLENGTH/2+j)%LL; /* use circular addressing */
            tmptmp1_40_fx=L_mac(tmptmp1_40_fx,x1_256_fx[m],sinc_fx[k][j]);
            tmptmp2_40_fx=L_mac(tmptmp2_40_fx,x2_256_fx[m],sinc_fx[k][j]);
        }
        tmptmp1_fx=round_fx(L_shl( tmptmp1_40_fx,1));
        tmptmp2_fx=round_fx(L_shl( tmptmp2_40_fx,1));

        out_fx[i]=round_fx(L_shl(L_mac(L_mult(tmptmp1_fx,sub(16384,w_fx)),tmptmp2_fx,w_fx),1));
    }


    free(tmp1_dtfs_fx);
    free(tmp2_dtfs_fx);
    free(tmp3_dtfs_fx);
}
/*===================================================================*/
/* FUNCTION      :  zeroFilter_fx()                                  */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  zero filtering                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) lpc[] :  lpc coefficients in Q12                     */
/*   _ (Word16) N     :  lpc order                                   */
/*   _ (Word16) this->lag:  in Q0                                    */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ None.                                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (Word16) this->a[] :    in Q(this->Q)                         */
/*   _ (Word16) this->b[] :    in Q(this->Q)                         */
/*   _ (Word16) this->Q:       in Q0                                 */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void DTFS_zeroFilter_fx( DTFS_STRUCTURE_FX *X_fx,Word16 *LPC, Word16 N, Word16 *S_fx, Word16 *C_fx)
{
    Word32 sum1_fx, sum2_fx ;
    Word16 k, n, HalfLag ;
    Word16 temp, temp1, temp2;
    Word32 L_temp1, L_temp2;
    Word16 Qmin, Qab[MAXLAG_WI], na, nb;

    Qmin = 32767;
    move16();
    HalfLag = s_min(shr(X_fx->lag_fx, 1),X_fx->nH_fx);
    FOR ( k=0 ; k<=HalfLag ; k++ )
    {
        temp = k;
        move16();
        temp2 = k;
        move16();

        /* Calculate sum1 and sum2 */
        sum1_fx = L_deposit_h(4096);      /*  1: Q(12+15+1)  */
        sum2_fx = L_deposit_l(0);

        FOR ( n=0 ; n<N ; n++ )
        {
            sum1_fx = L_mac(sum1_fx, LPC[n], C_fx[(4*temp2)%(4*X_fx->lag_fx)]) ;   /* Q(12+15+1) */
            sum2_fx = L_mac(sum2_fx, LPC[n], S_fx[(4*temp2)%(4*X_fx->lag_fx)]) ;
            temp2 = add(temp2, temp);
        }

        temp1 = round_fx(sum1_fx);   /* Q(12+15+1-16)=Q(12) */
        temp2 = round_fx(sum2_fx);   /*               Q(12) */

        /* Calculate the circular convolution */
        L_temp1 = L_mult(temp1, X_fx->a_fx[k]);
        L_temp1 = L_msu(L_temp1, temp2, X_fx->b_fx[k]); /*  Q(12+Q+1) */
        L_temp2 = L_mult(temp1, X_fx->b_fx[k]);
        L_temp2 = L_mac(L_temp2, temp2, X_fx->a_fx[k]); /*  Q(12+Q+1) */

        /*  normalization */
        na = norm_l(L_temp1);
        if (L_temp1==0)
        {
            na=31;
            move16();
        }
        nb = norm_l(L_temp2);
        if (L_temp2==0)
        {
            nb=31;
            move16();
        }

        if (sub(na, nb)<0)
        {
            nb=na;
            move16();
        }
        X_fx->a_fx[k] = round_fx((Word32)L_shl(L_temp1, nb));   /* Q(13+Q+nb-16)=Q(Q+nb-3) */
        X_fx->b_fx[k] = round_fx((Word32)L_shl(L_temp2, nb));   /*               Q(Q+nb-3) */

        Qab[k] = sub(nb, 3);

        if (sub(Qab[k], Qmin)<0)
        {
            Qmin = Qab[k];
            move16();
        }
    }
    /* bring to the same Q */
    FOR ( k=0 ; k<=HalfLag ; k++ )
    {
        X_fx->a_fx[k] = shl(X_fx->a_fx[k], sub(Qmin, Qab[k]));
        move16(); /*  Q(Q+Qab[k]+Qmin-Qab[k]=Q(Q+Qmin) */
        X_fx->b_fx[k] = shl(X_fx->b_fx[k], sub(Qmin, Qab[k]));
        move16(); /*                         Q(Q+Qmin) */
    }

    X_fx->Q = add(X_fx->Q, Qmin);
    move16();
}
/*===================================================================*/
/* FUNCTION      :  DTFS_poleFilter_fx()                                  */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  pole filtering                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) lpc[] :  lpc coefficients in Q12                     */
/*   _ (Word16) N     :  lpc order                                   */
/*   _ (Word16) lag:  in Q0                                          */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ None                                                          */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (Word16) this->a[] :    in Q(this->Q)                         */
/*   _ (Word16) this->b[] :    in Q(this->Q)                         */
/*   _ (Word16) this->Q:       in Q0                                 */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

/*  PORTING: Handling the functions with variable no. of arguments */

void DTFS_poleFilter_fx_9( DTFS_STRUCTURE_FX *X_fx, Word16 *pf_temp1, Word16 *pf_temp2, Word16 *pf_temp, Word16 *pf_n2_temp1)
{
    Word16 temp, temp1, temp2, HalfLag ;
    Word32 sum1_fx, sum2_fx;
    Word32 L_temp1, L_temp2;
    Word16 k,n2_temp1, na, nb;
    Word16 Qmin, Qab[MAXLAG_WI];

    Qmin = 32767;
    move16();
    HalfLag = s_min(shr(X_fx->lag_fx, 1),X_fx->nH_fx);
    FOR ( k=0; k<=HalfLag; k++ )
    {
        temp = temp2 = k;
        move16();
        move16();
        /* Calculate sum1 and sum2 */

        n2_temp1 = pf_n2_temp1[k];
        move16();

        temp1 = pf_temp1[k];
        move16();/* Q(12+15+1+n2-16)=Q(12+n2) */
        temp2 = pf_temp2[k];
        move16();/*                  Q(12+n2) */

        /* Calculate the circular convolution */

        L_temp1 = L_mac(L_mult(temp1, X_fx->a_fx[k]), temp2, X_fx->b_fx[k]); /*  Q(12+n2+Q+1)=Q(13+n2+Q) */
        L_temp2 = L_msu(L_mult(temp1, X_fx->b_fx[k]), temp2, X_fx->a_fx[k]); /*  Q(12+n2+Q+1)=Q(13+n2+Q) */

        temp = pf_temp[k];
        move16();                 /*  Q(61-25-2*n2-temp1-16)=Q(20-2*n2-temp1) */

        sum1_fx = Mult_32_16(L_temp1, temp);      /*  Q(13+n2+Q+15+exp-15) = Q(13+n2+Q+exp) */
        sum2_fx = Mult_32_16(L_temp2, temp);
        /*  normalization */
        na = norm_l(sum1_fx);
        if(sum1_fx==0)
        {
            na = 31;
            move16();
        }
        nb = norm_l(sum2_fx);
        if(sum2_fx==0)
        {
            nb = 31;
            move16();
        }

        if (sub(na, nb)<0)
        {
            nb=na;
            move16();
        }
        nb=sub(nb,1); /*  leave one more sign bit */
        X_fx->a_fx[k] = round_fx((Word32)L_shl(sum1_fx, nb));   /*  Q(-3+n2+Q+exp+nb ) */
        X_fx->b_fx[k] = round_fx((Word32)L_shl(sum2_fx, nb));

        Qab[k] = add(sub(nb, 3), n2_temp1);

        if (sub(Qab[k], Qmin)<0)
        {
            Qmin = Qab[k];
            move16();
        }
    }
    /*  bring to the same Q */
    move16();
    FOR ( k=0; k<=HalfLag; k++ )
    {
        X_fx->a_fx[k] = shl(X_fx->a_fx[k], sub(Qmin, Qab[k]));
        move16();       /*  Q(Q+Qab[k]+Qmin-Qab[k])=Q(Q+Qmin) */
        X_fx->b_fx[k] = shl(X_fx->b_fx[k], sub(Qmin, Qab[k]));
        move16();       /*  Q(Q+Qab[k]+Qmin-Qab[k])=Q(Q+Qmin) */
    }

    X_fx->Q = add(X_fx->Q, Qmin);
}
/*===================================================================*/
/* FUNCTION      :  DTFS_adjustLag_fx ()                             */
/*-------------------------------------------------------------------*/
/* PURPOSE       :                                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) N_fx:  lag value, Q0                                 */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (struct DTFS_fx) :    a/b in X1.Q                             */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (struct DTFS_fx) X1 :  lag in Q0                              */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void DTFS_adjustLag_fx(
    DTFS_STRUCTURE_FX *X_DTFS_FX,        /* i/o    : DTFS to adjust lag for    */
    Word16 N_fx                          /* i    : Target lag                  */
)
{
    Word32 en_fx;
    Word32 temp32_fx,tempnH_fx,mul1_fx,mul2_fx;
    Word16 k,diff_fx;
    Word16 exp,tmp;
    Word32 L_tmp;

    if (sub(N_fx,X_DTFS_FX->lag_fx)==0)
    {
        return ;
    }

    IF(sub(N_fx,X_DTFS_FX->lag_fx)>0)
    {
        DTFS_zeroPadd_fx(N_fx,X_DTFS_FX);
    }
    ELSE
    {
        en_fx = DTFS_getEngy_fx(X_DTFS_FX); /* Q = 2*(X_DTFS_FX->Q) */

        tmp = min(shr(X_DTFS_FX->lag_fx,1),X_DTFS_FX->nH_fx);
        FOR( k = add(shr(N_fx,1),1) ; k<= tmp ; k++)
        {
            X_DTFS_FX->a_fx[k] = 0;
            move16();
            X_DTFS_FX->b_fx[k] = 0;
            move16();
        }
        DTFS_setEngy_fx(X_DTFS_FX,en_fx);
        X_DTFS_FX->lag_fx = N_fx ;
        move16();


        /* recompute nH for new lag */
        exp = norm_s(X_DTFS_FX->lag_fx);
        tmp = div_s(shl(1,sub(14,exp)),X_DTFS_FX->lag_fx);/* 29 - exp */
        L_tmp = L_mult0(12800,tmp);
        temp32_fx = L_shl(L_tmp,sub(exp,23));
        diff_fx = (Word16) L_shl(L_tmp,sub(exp,29));


        exp = norm_s(diff_fx);
        tmp = div_s(shl(1,sub(14,exp)),diff_fx);/* 29 - exp */
        L_tmp = L_mult0(X_DTFS_FX->upper_cut_off_freq_fx,tmp);
        X_DTFS_FX->nH_fx = (Word16) L_shl(L_tmp,sub(exp,29));

        L_tmp = L_mult0(4000,tmp);
        tempnH_fx = L_shl(L_tmp,sub(exp,23));
        X_DTFS_FX->nH_4kHz_fx = extract_l(L_shl(L_tmp, sub(exp,29)));


        if(sub(sub(X_DTFS_FX->upper_cut_off_freq_fx, shr((Word16)L_mult(diff_fx,X_DTFS_FX->nH_fx),1)), diff_fx) >= 0)
        {
            X_DTFS_FX->nH_fx = add(X_DTFS_FX->nH_fx,1);
            move16();
        }

        mul1_fx = L_shl(temp32_fx,13);/* Q19 */
        mul2_fx = L_shl((Word32)X_DTFS_FX->nH_4kHz_fx,18);/* Q18 */
        tempnH_fx = Mult_32_32(mul1_fx,mul2_fx);/* Q6 */
        tempnH_fx = L_sub((Word32)256000,tempnH_fx);/* Q6 */

        if(L_sub(tempnH_fx,temp32_fx)>=0)
        {
            X_DTFS_FX->nH_4kHz_fx = add(X_DTFS_FX->nH_4kHz_fx,1);
            move16();
        }
    }
}
/*===================================================================*/
/* FUNCTION      :  DTFS_getEngy_fx ()                               */
/*-------------------------------------------------------------------*/
/* PURPOSE       :                                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) :    a/b in X_fx.Q i.e Q6, lag in Q0*/
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : _ None                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : _ None                                   */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*   _ (Word40) en_fx: output energy, 2*X1.Q                         */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/


Word32 DTFS_getEngy_fx( DTFS_STRUCTURE_FX *X_fx)
{
    Word16 k, HalfLag_fx;
    Word32 en_fx=0;
    Word16 temp_a_fx,temp_b_fx;

    HalfLag_fx = shr(sub(X_fx->lag_fx, 1), 1);
    HalfLag_fx = min(HalfLag_fx,X_fx->nH_fx);
    FOR (k=1; k<=HalfLag_fx; k++)
    {
        temp_a_fx = X_fx->a_fx[k];
        move16();
        temp_b_fx = X_fx->b_fx[k];
        move16();

        en_fx = L_mac0(en_fx, temp_a_fx, temp_a_fx);
        en_fx = L_mac0(en_fx, temp_b_fx, temp_b_fx);
    }

    en_fx = L_shr(en_fx, 1);
    temp_a_fx = X_fx->a_fx[0];
    en_fx = L_mac0(en_fx,temp_a_fx, temp_a_fx);

    /*  IF (X_fx->lag_fx%2 == 0)  */
    IF (s_and(X_fx->lag_fx,1) == 0)
    {
        temp_a_fx = X_fx->a_fx[k];
        move16();
        temp_b_fx = X_fx->b_fx[k];
        move16();

        en_fx = L_mac0(en_fx, temp_a_fx, temp_a_fx);
        en_fx = L_mac0(en_fx, temp_b_fx, temp_b_fx);
    }

    return en_fx ;  /*  2*X1.Q+1=Q13 */
}



/*===================================================================*/
/* FUNCTION      :  DTFS_getEngy_P2A_fx ()                           */
/*-------------------------------------------------------------------*/
/* PURPOSE       :                                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) :    a/b in X_fx.Q, lag in Q0      */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : _ None                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : _ None                                   */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*   _ (Word40) en_fx: output energy, 2*X1.Q                         */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

Word32 DTFS_getEngy_P2A_fx( DTFS_STRUCTURE_FX *X_fx)
{
    Word16 k, HalfLag_fx;
    Word32 en_fx=0;

    HalfLag_fx = shr(sub(X_fx->lag_fx, 1), 1);
    HalfLag_fx = s_min(HalfLag_fx,X_fx->nH_fx);
    FOR (k=1; k<=HalfLag_fx; k++)
    {
        en_fx = L_mac0(en_fx, X_fx->a_fx[k], X_fx->a_fx[k]);
        en_fx = L_mac0(en_fx,  X_fx->b_fx[k], X_fx->b_fx[k]);
    }
    en_fx = L_shr(en_fx, 1);
    en_fx = L_mac0(en_fx,X_fx->a_fx[0], X_fx->a_fx[0]);
    /* IF (X_fx->lag_fx%2 == 0) */
    IF (s_and(X_fx->lag_fx,1) == 0)
    {
        en_fx = L_mac0(en_fx, X_fx->a_fx[k], X_fx->a_fx[k]);
        en_fx = L_mac0(en_fx, X_fx->b_fx[k], X_fx->b_fx[k]);
    }

    return en_fx ;  /*  2*X1.Q */
}


/*=================================================================================*/
/* FUNCTION      :  DTFS_getEngy_band_fx (Word16 lband, Word16 hband)              */
/*---------------------------------------------------------------------------------*/
/* PURPOSE       :  compute the energy of X1.a[k] and X2.b[k]                      */
/*---------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                              */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx :  a_fx/b_fx in X_fx.Q, lag in Q0           */
/*   _ (Word16) lband:  Q0                                                         */
/*   _ (Word16) hband:  Q0                                                         */
/*---------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                              */
/*   _ (Word40) en_fx :       2*X1.Q                                             */
/*---------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                        */
/*                    _ None                                                       */
/*---------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                                      */
/*---------------------------------------------------------------------------------*/
/* CALLED FROM : TX                                                                */
/*=================================================================================*/

/*  PORTING: Handling the functions with variable no. of arguments */

Word32 DTFS_getEngy_band_fx(DTFS_STRUCTURE_FX X_fx,Word16 lband, Word16 hband)
{
    Word16 k, lk, hk, HalfLag ;
    Word32 freq_fx, L_lband, L_hband;
    Word32 en_fx=0;

    L_lband = L_mult(lband, X_fx.lag_fx);
    L_hband = L_mult(hband, X_fx.lag_fx);
    HalfLag = s_min(shr(sub(X_fx.lag_fx, 1), 1),X_fx.nH_4kHz_fx);
    /* get lband and hband */
    FOR (k=1; k<=HalfLag; k++)
    {
        freq_fx = L_mult(k, 12800);
        IF (L_sub(freq_fx, L_lband)>=0)
        {
            BREAK;
        }
    }
    lk = k;
    move16();
    FOR (k=1; k<=HalfLag; k++)
    {
        freq_fx = L_mult(k, 12800);
        IF (L_sub(freq_fx, L_hband)>=0)
        {
            BREAK;
        }
    }
    hk = k;
    move16();

    FOR (k=lk; k<hk; k++)
    {
        en_fx = L_mac0(en_fx, X_fx.a_fx[k], X_fx.a_fx[k]);          /*  2*X1.Q+1 */
        en_fx = L_mac0(en_fx, X_fx.b_fx[k], X_fx.b_fx[k]);
    }

    if (lband == 0)
    {
        en_fx = L_mac0(en_fx, X_fx.a_fx[0], X_fx.a_fx[0]) ;             /*  2*X1.Q+1 */
    }

    /* IF ((X_fx.lag_fx%2 == 0) && (hband == X_fx.upper_cut_off_freq_fx))  */
    test();
    IF ((s_and(X_fx.lag_fx , 1) == 0)&& (hband == X_fx.upper_cut_off_freq_fx))
    {
        en_fx = L_mac0(en_fx, X_fx.a_fx[k], X_fx.a_fx[k]);
        en_fx = L_mac0(en_fx, X_fx.b_fx[k], X_fx.b_fx[k]);
    }

    return en_fx ;                                     /*  2*X1.Q */
}
/*===================================================================*/
/* FUNCTION      :  DTFS_setEngy_fx ( )                              */
/*-------------------------------------------------------------------*/
/* PURPOSE       :                                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word40) en2_fx:        2*X1.Q+1 i.e. Q13                     */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word40) en1_fx :       2*X1.Q+1 i.e. Q13                     */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (struct DTFS_STRUCTURE_FX) X_DTFS_FX :  a/b in X1.Q i.e. Q6,  */
/*                    lag in Q0                                      */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

Word32 DTFS_setEngy_fx( DTFS_STRUCTURE_FX *X_DTFS_FX,Word32 en2_fx)
{
    Word16 k, HalfLag_fx ;
    Word32 en1_fx;
    Word32 L_temp_fx;
    Word16 expa,expb,fraca,fracb,scale,tmp,exp;
    Word32 L_tmp,factor_fx;

    HalfLag_fx = min(shr(X_DTFS_FX->lag_fx, 1),X_DTFS_FX->nH_fx);
    move16();
    en1_fx = DTFS_getEngy_fx(X_DTFS_FX);
    move16();

    if (en1_fx == 0)
    {
        return 0;
    }

    expa = norm_l(en2_fx);
    fraca = extract_h(L_shl(en2_fx,expa));
    expa = sub(30, add(expa,shl(X_DTFS_FX->Q, 1)));


    expb = norm_l(en1_fx);
    fracb = round_fx(L_shl(en1_fx,expb));
    expb = sub(30, add(expb, shl(X_DTFS_FX->Q, 1)));


    scale = shr(sub(fraca,fracb),15);
    fracb = shl(fracb,scale);
    expb = sub(expb,scale);

    tmp = div_s(fracb,fraca);
    exp = sub(expb,expa);

    L_tmp = Isqrt_lc(L_deposit_h(tmp),&exp); /* Q(31-exp) */
    factor_fx = L_shr(L_tmp,1);

    FOR (k=0; k<=HalfLag_fx; k++)
    {
        L_temp_fx = Mult_32_16(factor_fx, X_DTFS_FX->a_fx[k]);              /*  Q(temp+X1.Q-15) */
        X_DTFS_FX->a_fx[k] = round_fx(L_temp_fx);                  /*  Q(temp+X1.Q-15-16)=Q(temp+X1.Q-31); */

        L_temp_fx =Mult_32_16(factor_fx, X_DTFS_FX->b_fx[k]);               /*  Q(temp+X1.Q-15) */
        X_DTFS_FX->b_fx[k] = round_fx(L_temp_fx);                  /*  Q(temp+X1.Q-15-16)=Q(temp+X1.Q-31); */

    }


    return en1_fx ;/*  2*X1.Q+1 = Q13 */
}

/*===================================================================*/
/* FUNCTION      :  struct DTFS_car2pol_fx ()                        */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Cartesian to polar representation                */
/*                  returning amplitudes and 0 phases                */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx :  prototype in Cartesian domain*/
/*         (Word16) lag: length of prototype in time domain          */
/*         (Word16 []) a,b: re/im of harmonics, normalized            */
/*         (Word16) Q: norm factor of a/b                             */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx :  prototype in polar domain  */
/*          (Word16) lag: length of prototype in time domain         */
/*          (Word16 []) a: amplitude of harmonics, normalized        */
/*          (Word16 []) b: phase of harmonics,cleared to 0           */
/*          (Word16) Q: norm factor of a                             */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
/* NOTE: output X.b (angle) is not computed and should be ignored    */
/*       When phases are needed, such as in QPPP, instead uses the   */
/*       Cartesian representation to avoid computing phases by arctan*/
/*===================================================================*/
void DTFS_car2pol_fx(
    DTFS_STRUCTURE_FX *X_fx       /* i/o : DTFS structure a, b, lag  */
)

{
    Word16 k, HalfLag_fx ;
    Word32 Ltemp_fx;
    Word32 Lacc_fx;
    Word16 exp,tmp,frac;

    HalfLag_fx = s_min(shr(sub(X_fx->lag_fx,1),1),X_fx->nH_fx);
    FOR ( k=1 ; k<=HalfLag_fx; k++ )
    {

        Lacc_fx=L_mult(X_fx->a_fx[k],X_fx->a_fx[k]); /*  a[k]^2, 2Q+1 */
        Lacc_fx=L_mac(Lacc_fx,X_fx->b_fx[k],X_fx->b_fx[k]); /*  a[k]^2+b[k]^2, 2Q+1 */
        Lacc_fx=L_shr(Lacc_fx,3); /*  Lacc=(a[k]^2+b[k]^2)/4, 2Q */

        IF(Lacc_fx)
        {
            exp = norm_l(Lacc_fx);
            frac = extract_h(L_shl(Lacc_fx,exp)); /* Q14 */
            exp = sub(exp, sub(30,(2*X_fx->Q)));

            tmp = div_s(16384,frac); /* Q15 */
            Ltemp_fx = Isqrt_lc(L_deposit_h(tmp),&exp); /* Q(31-exp) */

            X_fx->a_fx[k]=extract_h(L_shl(Ltemp_fx, sub(add(X_fx->Q, exp), 15)));    /*  Q */
        }
        ELSE
        {
            X_fx->a_fx[k] = 0;
            move16();
        }

        X_fx->b_fx[k]= 0;
        move16(); /*  clear b[k] */
    }


    IF (s_and(X_fx->lag_fx,1) == 0)
    {
        IF(X_fx->a_fx[k])
        {
            Lacc_fx=L_mult0(X_fx->a_fx[k],X_fx->a_fx[k]); /*  a[k]^2, 2Q+1 */
            Lacc_fx=L_mac0(Lacc_fx,X_fx->b_fx[k],X_fx->b_fx[k]); /*  a[k]^2+b[k]^2, 2Q+1 */

            exp = norm_l(Lacc_fx);
            frac = extract_h(L_shl(Lacc_fx,exp)); /* Q14 */
            exp = sub(exp,sub(30,shl(X_fx->Q,1)));

            tmp = div_s(16384,frac); /* Q15 */
            Ltemp_fx = Isqrt_lc(L_deposit_h(tmp),&exp); /* Q(31-exp) */

            X_fx->a_fx[k]=extract_h(L_shl(Ltemp_fx, sub(add(X_fx->Q, exp), 15))); /*  Q */
        }
        ELSE
        {
            X_fx->a_fx[k] = 0;
            move16();
        }

        X_fx->b_fx[k]= 0;
        move16(); /*  clear b[k];   */
    }
}


/*==============================================================================*/
/* FUNCTION      :  DTFS_setEngyHarm_fx ( )                                     */
/*------------------------------------------------------------------------------*/
/* PURPOSE       :  Set a band of harmonics to specified energy                 */
/*----------------------------------------------------------------------- ------*/
/* INPUT ARGUMENTS  :                                                            */
/*   _ (struct DTFS_fx) : lag in Q0                                              */
/*   _ (Word16) f1_fx:   lower bound of input, normalized by 12800, Q15          */
/*   _ (Word16) f2_fx:   upper bound of input, normalized by 12800, Q15          */
/*   _ (Word16) g1_fx:   lower bound of output, normalized by 12800, Q15         */
/*   _ (Word16) g2_fx:   upper bound of output, normalized by 12800, Q15         */
/*   _ (Word32) en2_fx:  in Q(Qen2)                                              */
/*   _ (Word16) Qen2_fx: Q value of en2                                          */
/*---------------------------------------------------------------------------- --*/
/* OUTPUT ARGUMENTS :                                                            */
/*   _ (Word16) Qa_fx: Q value of output a[].                                    */
/*----------------------------------------------------------------------------- -*/
/* INPUT/OUTPUT ARGUMENTS :                                                      */
/* _(struct DTFS_STRUCTURE_FX) : a_fx[] in X1_fx.Q at start, then changed to     */
/*                    *Qa later.                                                 */
/*---------------------------------------------------------------------------- --*/
/* RETURN ARGUMENTS : _ None.                                                    */
/*   _ (Word32) en1_fx:  Q(2*X1.Q)                                               */
/*----------------------------------------------------------------------------- -*/
/* CALLED FROM : TX/RX                                                           */
/*============================================================================== */
/* NOTE: This function cannot change Q because it works on a band                */
/*       of harmonics, instead of the whole DTFS                                 */
/*============================================================================== */
Word32 DTFS_setEngyHarm_fx(
    Word16 f1_fx,                /* i  : lower band freq of input to control energy   */
    Word16 f2_fx,                /* i  : upper band freq of input to control energy   */
    Word16 g1_fx,                /* i  : lower band freq of output to control energy  */
    Word16 g2_fx,                /* i  : upper band freq of output to control energy  */
    Word32 en2_fx,               /* i  : Target Energy to set the DTFS to             */
    Word16 Qen2_fx,              /* i  : Input Q format for en2                         */
    Word16 *Qa_fx,               /* i  : Output Q format for x->a                     */
    DTFS_STRUCTURE_FX *X_fx      /* i/o: DTFS to adjust the energy of                 */
)
{

    Word16 k, count=0, HalfLag_fx;
    Word16 f_low_fx, f_high_fx, g_low_fx, g_high_fx;
    Word32 L_temp_fx, factor_fx;
    Word32 en1_fx;
    Word32 Lacc;
    Word16 exp,tmp,expa,expb,fraca,fracb,scale;
    Word32 L_tmp;



    f_low_fx=mult(f1_fx,X_fx->lag_fx); /*  Q0 */
    f_high_fx=mult(f2_fx,X_fx->lag_fx); /*  Q0 */
    g_low_fx=mult(g1_fx,X_fx->lag_fx); /*  Q0 */
    g_high_fx=mult(g2_fx,X_fx->lag_fx); /*  Q0 */
    HalfLag_fx = s_min(f_high_fx,shl(X_fx->nH_fx,1));

    Lacc = L_deposit_l(0);
    FOR (k=f_low_fx+1 ; k<=HalfLag_fx; k++)
    {
        Lacc = L_mac0(Lacc, X_fx->a_fx[k], X_fx->a_fx[k]);         /*  2*X1.Q */
        count=add(count,1);
    }
    exp = norm_s(count);
    tmp = div_s(shl(1,sub(14,exp)),count);/* 29 - exp */
    en1_fx = L_shl( Mult_32_16(Lacc , tmp), sub(exp,14));

    test();
    IF (en1_fx>0 && en2_fx > 0)
    {
        /* factor_fx = sqrt_divide_dp((Word40)en2_fx, en1_fx, sub(Qen2_fx, shl(X_fx->Q, 1)), &temp_fx,1);      : Q(temp) */
        expa = norm_l(en2_fx);
        fraca = extract_h(L_shl(en2_fx,expa));
        expa = sub(30,add(expa,Qen2_fx));


        expb = norm_l(en1_fx);
        fracb = round_fx(L_shl(en1_fx,expb));
        expb = sub(30, add(expb, shl(X_fx->Q, 1)));


        scale = shr(sub(fraca,fracb),15);
        fracb = shl(fracb,scale);
        expb = sub(expb,scale);

        tmp = div_s(fracb,fraca);
        exp = sub(expb,expa);

        L_tmp = Isqrt_lc(L_deposit_h(tmp),&exp); /* Q(31-exp) */
        factor_fx = L_shr(L_tmp,1);

    }
    ELSE
    {
        factor_fx = L_deposit_l(0);
    }

    HalfLag_fx = s_min(g_high_fx,shl(X_fx->nH_fx,1));
    FOR (k=g_low_fx+1; k<=HalfLag_fx; k++)
    {
        /*L_temp_fx =(Word32)Mpy_32_16(extract_h(factor_fx),extract_l(factor_fx), X_fx->a_fx[k]);  move32();           */ /*  Q(temp+X1.Q-15) */
        L_temp_fx = Mult_32_16(factor_fx, X_fx->a_fx[k]);      /*  Q(temp+X1.Q-15) */
        X_fx->a_fx[k] = round_fx(L_temp_fx);  /*  Q(temp+X1.Q-15-16)=Q(temp+X1.Q-31); */
    }

    *Qa_fx = sub( sub(X_fx->Q,1) , exp);

    return en1_fx;       /*  Q(2*X1.Q) */


}
/*===================================================================*/
/* FUNCTION      :  cubicPhase_fx ()                                 */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Compute cubic phase track for WI synthesis       */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) ph1_fx :  initial phase, Q15 (normalized by 2pi)     */
/*   _ (Word16) ph2_fx :  final phase, Q15 (normalized by 2pi)       */
/*   _ (Word16) L1 :  previous pitch lag, Q0                         */
/*   _ (Word16) L2 :  current pitch lag, Q0                          */
/*   _ (Word16) N :  length of phase track, Q0                       */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word32 []) phOut_fx :  phase track, Q27 (normalized by 2pi)  */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
/* NOTE: This function outputs phase in (-1,1)                       */
/*===================================================================*/
static
void cubicPhase_fx(
    Word16 ph1_fx, Word16 ph2_fx, Word16 L1, Word16 L2,
    Word16 N, Word32 *phOut_fx
)
{
    Word16 n;
    Word16 n2;
    Word16 f1, f2;  /*  invert of L1, L2, Q19 */
    Word16 factor, temp;
    Word16 c0, c1, c2, c3; /*  cubic polynomial coefficients */
    /*  Q33, Q27, Q19, Q15 respectively */
    Word32 Ltemp1, Ltemp2, Ltemp3,Ltemp4,Ltemp;
    Word16 tmp,exp;
    Word32 Lacc;
    Word16 expa,expb,fraca,fracb,scale;
    Word32 L_tmp,L_tmp1;
    Word16 num_flag,den_flag;
    Word32 N2;
    Word16 dbgshft;
    num_flag = 0;
    den_flag = 0;

    N=sub(N,L2);

    exp = norm_s(L1);
    tmp = div_s(shl(1,sub(14,exp)),L1);
    L_tmp = L_shl(tmp,exp+6);
    f1 = round_fx(L_tmp);

    exp = norm_s(L2);
    tmp = div_s(shl(1,sub(14,exp)),L2);
    L_tmp = L_shl(tmp,exp+6);
    Ltemp4 = inverse_table[L2];
    f2 = round_fx(L_tmp);

    c3 = ph1_fx;
    move16(); /*  c3 in Q15 */
    c2 = f1;
    move16();/*  c2 in Q19 */

    Ltemp1 = L_sub(ph1_fx,ph2_fx); /*  Ltemp1=ph1_fx-ph2_fx, Q15 */
    Ltemp2 = L_add(f1,f2); /*  Ltemp2=0.5*(f1+f2), Q20 */
    temp = extract_l(Ltemp2);/* Q20 */

    IF(temp<0)
    {
        L_tmp1 = L_shl(L_add(65536,temp),14); /* Q30 */
        L_tmp = Mult_32_16(L_tmp1,N); /* 30-15=15 */
        Ltemp2 = L_shl(L_tmp,1);
    }
    ELSE
    {
        Ltemp2 = (Word32)L_mult0(N,temp); /*  Ltemp2=0.5*N*(f1+f2), Q20 */
    }

    Ltemp1 = L_add(L_shl(Ltemp1,5),Ltemp2); /*  Ltemp1=ph1_fx - ph2_fx + 0.5*N*(f2+f1), Q16, Q20 */

    factor = round_fx(L_shr(Ltemp1,4));  /*  factor in Q0 */

    c1 = sub(f2,f1);  /*  c1=f2-f1, Q19 */
    Ltemp1 = L_sub(ph2_fx,ph1_fx); /*  Q15 */
    Ltemp2 = L_mult(N,f1); /*  Ltemp2=N*f1, Q20 */
    Ltemp2 = L_sub(L_shl(L_deposit_h(factor),4),Ltemp2); /*  Ltemp2=factor-N*f1, Q20 */
    Ltemp1 = L_add(Ltemp2,L_shl(Ltemp1,5)); /*  Ltemp1 in Q20 */

    IF(sub(N,180)>0)
    {
        Ltemp2 = L_shl(L_mult0(N,N),14);
        Ltemp2 = L_shl(Mult_32_16(Ltemp2,N),1);

        /* IF(N%2) */
        if(s_and(N,1) == 1)
        {
            Ltemp2 = L_add(Ltemp2,1);
        }
    }
    ELSE
    {
        Ltemp2=L_shr(L_mult(N,N),1);
        Ltemp2 = L_mult0(N,extract_l(Ltemp2)); /*  Ltemp2=N^3 */
    }
    Ltemp3 = L_mult(N,c1); /*  Q20 */
    Ltemp3 = L_sub(Ltemp3,L_shl(Ltemp1,1)); /*  Ltemp3=N*c1-2*Ltemp1, Q20 */


    IF (L_sub(L_abs(Ltemp3),L_shl(Ltemp2,8)) >= 0)
    {
        Lacc = L_add(MIN_32, 0);
        if (Ltemp3 > 0)
        {
            Lacc = L_add(MAX_32, 0);
        }

        c0 = extract_h(Lacc); /*  c0 in Q33 */
    }
    ELSE
    {
        expa = norm_l(Ltemp3);
        fraca = extract_h(L_shl(Ltemp3,expa));
        expa = sub(30,add(expa, 20));
        if(fraca<0)
        {
            num_flag = 1;
            move16();
        }

        expb = norm_l(Ltemp2);
        fracb = extract_h(L_shl(Ltemp2,expb));
        expb =  sub(30,expb);
        if(fracb<0)
        {
            den_flag = 1;
            move16();
        }

        if(num_flag)
        {
            fraca = negate(fraca);
        }
        if(den_flag)
        {
            fracb = negate(fracb);
        }
        scale = shr(sub(fracb,fraca),15);
        fraca = shl(fraca,scale);
        expa = sub(expa,scale);

        tmp = div_s(fraca,fracb); /* 15-exp */
        exp = sub(expa,expb);
        test();
        if(num_flag && !den_flag)
        {
            tmp = negate(tmp);
        }
        test();
        if(den_flag && !num_flag)
        {
            tmp = negate(tmp);
        }

        Lacc = L_shl(tmp,add(exp,34));
        Lacc = L_add(Lacc,0x08000);
        c0 = extract_h(Lacc); /*  c0 in Q33 */
    }

    Ltemp1 = L_mult(N,N); /*  Ltemp1=2*N*N */
    Ltemp1 = L_add(Ltemp1,L_shr(Ltemp1,1)); /*  Ltemp1=3*N*N, max is 3*140*140 */

    /* patch added for time warping support, where N can be more than 140 */
    dbgshft= norm_l(Ltemp1);
    Ltemp1= L_shl(Ltemp1,dbgshft);
    temp = extract_h(Ltemp1);
    Ltemp1 = (Word32)L_shl((Word32)L_mult0(c0,temp), sub(16,dbgshft)); /*  Ltemp1=3*N*N*c0, Q33 */
    /* Patch end */

    num_flag = den_flag = 0;
    move16();
    move16();
    Ltemp1 = L_sub(L_shr(L_deposit_h(c1),2),Ltemp1); /*  Ltemp1=c1-3*N*N*c0, Q33 */

    expa = norm_l(Ltemp1);
    fraca = extract_h(L_shl(Ltemp1,expa));
    expa = sub(30,add(expa, 33));
    if(fraca<0)
    {
        num_flag = 1;
        move16();
    }

    expb = norm_l(N);
    fracb = extract_h(L_shl(N,expb));
    expb =  sub(30,expb);
    if(fracb<0)
    {
        den_flag = 1;
        move16();
    }

    if(num_flag)
    {
        fraca = negate(fraca);
    }
    if(den_flag)
    {
        fracb = negate(fracb);
    }
    scale = shr(sub(fracb,fraca),15);
    fraca = shl(fraca,scale);
    expa = sub(expa,scale);

    tmp = div_s(fraca,fracb); /* 15-exp */
    exp = sub(expa,expb);
    test();
    if(num_flag && !den_flag)
    {
        tmp = negate(tmp);
    }
    test();
    if(den_flag && !num_flag)
    {
        tmp = negate(tmp);
    }

    Lacc = L_shl(tmp,exp+27);
    Lacc = L_add(Lacc,0x08000);
    c1 = extract_h(Lacc); /*  c1 in Q27 */


    /*  Computation of the phase value at each sample point */
    /*  ph[n]=  c0*n^3+c1*n^2+c2*n+c3, Q15 */
    phOut_fx[0] = L_shl(ph1_fx,11);/* Q27 */

    IF(sub(N,181) < 0)
    {
        FOR (n=1; n<N; n++)
        {
            /*     phOut_fx[n] = _POLY3(n,coef) ; */
            n2 = i_mult2(n,n); /*  n2=n^2 */
            Ltemp3 = (Word32)L_mult0(n,(UNS_Word16)n2); /*  Ltemp3=n^3 */

            Ltemp3 = L_shl(Mult_32_16(L_shl(Ltemp3,7),c0),2); /*  Ltemp3=c0*n^3, Q27 */
            Ltemp2 = (Word32)L_mult0(c1,(UNS_Word16)n2); /*  Ltemp2=c1*n^2, Q27 */
            Ltemp1 = L_shl(L_mult(c2,n),7);  /*  Ltemp1=c2*n, Q27 */

            Ltemp = L_shl((Word32)c3,12);/* Q27 */
            Ltemp = L_add(Ltemp1,Ltemp);/* Q27 */
            Ltemp = L_add(Ltemp2,Ltemp); /* Q27 */
            Ltemp = L_add(Ltemp3,Ltemp); /*  Q27 */

            phOut_fx[n] = Ltemp;
            move32(); /* Q27 */
        }
    }
    ELSE
    {

        FOR (n=1; n<181; n++)
        {
            /*     phOut_fx[n] = _POLY3(n,coef) ; */
            n2 = i_mult2(n,n); /*  n2=n^2 */
            Ltemp3 = (Word32)L_mult0(n,(UNS_Word16)n2); /*  Ltemp3=n^3 */

            Ltemp3 = L_shl(Mult_32_16(L_shl(Ltemp3,7),c0),2); /*  Ltemp3=c0*n^3, Q27 */
            Ltemp2 = (Word32)L_mult0(c1,(UNS_Word16)n2); /*  Ltemp2=c1*n^2, Q27 */
            Ltemp1 = L_shl(L_mult(c2,n),7);  /*  Ltemp1=c2*n, Q27 */

            Ltemp = L_shl((Word32)c3,12);/* Q27 */
            Ltemp = L_add(Ltemp1,Ltemp);/* Q27 */
            Ltemp = L_add(Ltemp2,Ltemp); /* Q27 */
            Ltemp = L_add(Ltemp3,Ltemp); /*  Q27 */

            phOut_fx[n] = Ltemp;
            move32(); /* Q27 */
        }

        FOR (n=181; n<N; n++)
        {
            /*     phOut_fx[n] = _POLY3(n,coef) ; */
            N2 = L_shl(L_mult0(n,n),14);
            Ltemp3 = L_shl(Mult_32_16(N2,n),1);

            if(s_and(N,1) == 1)
            {
                Ltemp3 = L_add(Ltemp3,1);
            }

            Ltemp3 = L_shl(Mult_32_16(L_shl(Ltemp3,7),c0),2); /*  Ltemp3=c0*n^3, Q27 */

            Ltemp2 = L_shl(Mult_32_16(N2 ,c1 ),1);
            /*  Ltemp2 = (Word32)L_mult_su(c1,(UNS_Word16)n2); : Ltemp2=c1*n^2, Q27 */
            Ltemp1 = L_shl(L_mult(c2,n),7);  /*  Ltemp1=c2*n, Q27 */

            Ltemp = L_shl((Word32)c3,12);/* Q27 */
            Ltemp = L_add(Ltemp1,Ltemp);/* Q27 */
            Ltemp = L_add(Ltemp2,Ltemp); /* Q27 */
            Ltemp = L_add(Ltemp3,Ltemp); /*  Q27 */

            phOut_fx[n] = Ltemp;
            move32(); /* Q27 */
        }
    }

    tmp = add(N,L2);
    FOR (; n<tmp; n++)
    {
        Ltemp = L_add(phOut_fx[n-1],Ltemp4);/* Q27 */
        phOut_fx[n] = Ltemp;
        move32(); /* Q27 */

    }

}

/*===================================================================*/
/* FUNCTION      :   DTFS_to_erb_fx ()                               */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Convert harmonics to erb bands                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx :  prototype in polar domain  */
/*                (Word16) lag_fx: length of prototype in time domain*/
/*                (Word16 []) a_fx: amplitude, normalized            */
/*                (Word16) Q_fx: norm factor of a                    */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) out_fx : erb output, Q13                          */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
void DTFS_to_erb_fx(
    const DTFS_STRUCTURE_FX X_fx,            /* i : DTFS input       */
    Word16 *out_fx                           /* o : ERB output       */
)

{
    Word16 num_erb_fx;
    Word16  i, j, n, count[NUM_ERB_WB] ;
    Word16 diff_fx ;
    Word32 sum_a_fx[NUM_ERB_WB],Ltemp_fx,L_tmp,L_temp;
    Word16 exp,tmp;
    Word16 expa,expb,fraca,fracb,scale;

    const Word16 *erb_fx = NULL;
    num_erb_fx=NUM_ERB_NB;
    move16();

    test();
    IF (sub(X_fx.upper_cut_off_freq_fx, 0x02800)== 0 || sub(X_fx.upper_cut_off_freq_fx, 4000) == 0 )/*  0x2800=0.3125 in Q15 (4000Hz) */
    {
        num_erb_fx=NUM_ERB_NB;
        move16();
        erb_fx=&(erb_NB_fx[0]);
        move16();
    }
    ELSE IF (sub(X_fx.upper_cut_off_freq_fx, 0x04000) == 0 || sub(X_fx.upper_cut_off_freq_fx, 6400) == 0)/*  0x4000=0.5 in Q15 (6400Hz) */
    {
        test();
        num_erb_fx=NUM_ERB_WB;
        move16();
        erb_fx=&(erb_WB_fx[0]);
        move16();
    }


    FOR (i=0; i<num_erb_fx; i++)
    {


        count[i] = 0 ;
        move16();
        sum_a_fx[i] = L_deposit_l(0);
    }

    exp = norm_s(X_fx.lag_fx);
    tmp = div_s(shl(1,sub(14,exp)),X_fx.lag_fx);/* 29-exp */

    L_tmp = L_shl(tmp, add(exp,6));
    diff_fx = round_fx(L_tmp);

    n = s_min(shr(X_fx.lag_fx,1),X_fx.nH_fx);

    j=0;
    move16();
    FOR (i=0 ; i<=n; i++)
    {
        Ltemp_fx=L_mult(diff_fx,i);  /*  Ltemp=i*diff, Q20 */
        FOR ( ; j<num_erb_fx; j++)
        {
            IF (L_sub(Ltemp_fx,L_shl(erb_fx[j+1],5))<0)
            {
                sum_a_fx[j] = L_add(sum_a_fx[j],L_deposit_l(X_fx.a_fx[i]));
                move32();/*  X_fx.Q */
                count[j]=add(count[j],1);
                move16();
                BREAK ;
            }
        }
    }
    /* Make output in Q13  */
    n=sub(29,X_fx.Q);
    j=negate(X_fx.Q);



    FOR (i=0; i<num_erb_fx; i++)
    {
        out_fx[i]=round_fx(L_shl(sum_a_fx[i],n));  /*  Q13 */

        IF (sub(count[i],1)>0)
        {
            IF(sum_a_fx[i]<0)
            {
                L_temp = L_negate(sum_a_fx[i]);
            }
            ELSE
            {
                L_temp = L_add(sum_a_fx[i], 0);
            }

            expb = norm_l(L_temp);
            fracb = round_fx(L_shl(L_temp,expb));
            expb = sub(30, add(expb,X_fx.Q));


            expa = norm_l(count[i]);
            fraca = extract_h(L_shl(count[i],expa));
            expa = sub(30,expa);

            scale = shr(sub(fraca,fracb),15);
            fracb = shl(fracb,scale);
            expb = sub(expb,scale);

            tmp = div_s(fracb,fraca);
            exp = sub(expb,expa);
            L_tmp = L_shl(tmp, add(exp,14));

            out_fx[i] = round_fx(L_tmp);
        }
    }
}
/*===================================================================*/
/* FUNCTION      :  erb_slot_fx ()                                   */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Allocate harmonics in ERB bins                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) lag_fx :  pitch lag, Q0                              */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) out_fx : number of harmonics in the ERB bins, Q0  */
/*   _ (Word16 []) mfreq_fx : frequency bounds of the ERB bins, Q15  */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
/* NOTE: Frequency is normalized by 12800, i.e. 1=12800Hz            */
/*===================================================================*/
void erb_slot_fx(
    Word16   lag_fx,        /* i : input lag          */
    Word16   *out_fx,       /* o : ERB slots          */
    Word16 *mfreq_fx,       /* i : ERB frequencies    */
    Word16 num_erb_fx       /* i : number of ERBs     */
)
{
    Word16 i, j,n ;
    Word16 diff_fx;
    Word16 upper_cut_off_freq_fx;
    Word32 Ltemp_fx;
    Word32 mf_fx[NUM_ERB_WB];
    Word16 nH_band_fx;
    Word16 exp,tmp;
    Word32 L_tmp1,L_tmp;
    Word16 fraca,fracb,expa,expb,scale;

    const Word16 *erb_fx=NULL;/*move16(); */

    upper_cut_off_freq_fx=4000;
    move16();

    IF (sub(num_erb_fx,NUM_ERB_NB)==0)
    {
        upper_cut_off_freq_fx=4000;
        move16();
        erb_fx=&(erb_NB_fx[0]);
        move16();
    }
    ELSE IF (sub(num_erb_fx,NUM_ERB_WB)==0)
    {
        upper_cut_off_freq_fx=6400;
        move16();
        erb_fx=&(erb_WB_fx[0]);
        move16();
    }

    exp = norm_s(lag_fx);
    tmp = div_s(shl(1, sub(14,exp)),lag_fx); /* Q29-exp */
    L_tmp1 = L_mult(12800,tmp); /* Q(30-exp) */
    diff_fx = extract_h(L_shl(L_tmp1,sub(exp,14))); /* Q0 */

    exp = norm_s(diff_fx);
    tmp = div_s(shl(1,sub(14,exp)),diff_fx); /* Q29-exp */
    L_tmp1 = L_mult(upper_cut_off_freq_fx,tmp); /* Q(30-exp) */
    nH_band_fx = round_fx(L_shl(L_tmp1,sub(exp,14))); /* Q0 */

    FOR (i=0; i<num_erb_fx; i++)
    {
        out_fx[i] = 0 ;
        move16();
        mf_fx[i] = 0 ;
        move16();
    }


    L_tmp = L_mult0(diff_fx,nH_band_fx);/* Q0 */


    if(L_sub(upper_cut_off_freq_fx, L_tmp)>=diff_fx)/* Q0 compare */
    {
        nH_band_fx = add(nH_band_fx,1);/* Q0 */
    }

    n=s_min(shr(lag_fx,1),nH_band_fx);
    exp = norm_s(lag_fx);
    tmp = div_s(shl(1,sub(14,exp)),lag_fx); /* Q29-exp */
    L_tmp = L_shl(tmp, add(exp,6));
    diff_fx =round_fx(L_tmp);

    j=0;
    move16();
    FOR (i=0; i<=n; i++)
    {
        Ltemp_fx=L_mult(diff_fx,i); /*  Ltemp=i*diff, Q20 */
        /* freq=round32_16(L_shl(Ltemp,11)); : freq=i*diff, Q15 */

        IF (sub(num_erb_fx,NUM_ERB_NB)==0)
        {
            Ltemp_fx=L_min(Ltemp_fx,0x050000); /*  0x50000=0.3125 in Q20 (4000Hz) */
        }
        ELSE IF (sub(num_erb_fx,NUM_ERB_WB)==0)
        {
            Ltemp_fx=L_min(Ltemp_fx,0x080000); /*  0x80000=0.5 in Q20 (6400Hz)     */
        }

        FOR ( ; j<num_erb_fx; j++)
        {
            IF (L_sub(Ltemp_fx,L_shl(erb_fx[j+1],5))<0)
            {
                mf_fx[j] = L_add(mf_fx[j],Ltemp_fx);
                move32();
                out_fx[j]=add(out_fx[j],1);
                move16();
                BREAK ;
            }
        }
    }
    FOR (j=0; j<num_erb_fx; j++)
    {
        mfreq_fx[j]=round_fx(L_shl(mf_fx[j],11)); /*  Q15 */

        IF (sub(out_fx[j],1)>0)
        {
            expb = norm_l(mf_fx[j]);
            fracb = round_fx(L_shl(mf_fx[j],expb));
            expb = sub(30, add(expb,20));


            expa = norm_l(out_fx[j]);
            fraca = extract_h(L_shl(out_fx[j],expa));
            expa = sub(30,expa);

            scale = shr(sub(fraca,fracb),15);
            fracb = shl(fracb,scale);
            expb = sub(expb,scale);

            tmp = div_s(fracb,fraca);
            exp = sub(expb,expa);
            L_tmp = L_shl(tmp,add(exp,16));

            mfreq_fx[j] = round_fx(L_tmp);

        }
    }

}
/*===================================================================*/
/* FUNCTION      :   DTFS_erb_inv_fx ()                              */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Convert erb into harmonics                       */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16 []) in_fx : erb output, Q13                           */
/*   _ (Word16 []) slot_fx : number of harmonics in the ERB bins, Q0 */
/*   _ (Word16 []) mfreq_fx : frequency bounds of the ERB bins, Q15  */
/*   _ (struct DTFS_STRUCTURE_FX) (Word16) lag_fx: length of         */
/*                                       prototype in time domain    */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) : prototype in polar domain        */
/*                (Word16 []) a_fx: amplitude, normalized            */
/*                (Word16) Q: norm factor of a                       */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void DTFS_erb_inv_fx(
    Word16 *in_fx,                   /* i : ERB inpt                      */
    Word16   *slot_fx,               /* i : ERB slots filled based on lag */
    Word16 *mfreq_fx,                /* i : erb frequence edges           */
    DTFS_STRUCTURE_FX *X_fx,         /* o : DTFS after erb-inv             */
    Word16 num_erb_fx                /* i : Number of ERB bands             */
)
{

    Word16 i, j, m_fx=0,n,HalfLag_fx;
    Word16 diff_fx;/* 1/lag, Q19 */
    Word16 d1,d2, q[MAXLAG_WI], min_q=0;
    Word16 d1h, d1l, d2h, d2l;
    Word16 freq_fx, f_fx[NUM_ERB_WB+2], amp_fx[NUM_ERB_WB+2] ;
    Word16 upper_cut_off_freq_fx = 0;
    Word32 Ltemp_fx,Ltemp2_fx;
    Word32 Lacc_fx;
    Word16 exp,tmp;


    IF (sub(num_erb_fx,NUM_ERB_NB)==0)
    {
        upper_cut_off_freq_fx=0x02800;
        move16();/*  0x2800=0.3125 in Q15 (4000Hz) */
    }
    ELSE IF (sub(num_erb_fx,NUM_ERB_WB)==0)
    {
        upper_cut_off_freq_fx=0x04000;
        move16();/*  0x4000=0.5 in Q15 (6400Hz) */
    }

    f_fx[m_fx]=0;
    move16();
    amp_fx[m_fx]=0;
    move16();
    m_fx=add(m_fx,1);

    FOR (i=0; i<num_erb_fx; i++)
    {
        IF (slot_fx[i] != 0)
        {
            f_fx[m_fx]=mfreq_fx[i];
            move16();
            amp_fx[m_fx]=in_fx[i];
            move16();
            m_fx=add(m_fx,1);
        }
    }
    f_fx[m_fx]=upper_cut_off_freq_fx;
    move16();
    amp_fx[m_fx]=0;
    move16();
    m_fx=add(m_fx,1);

    exp = norm_s(X_fx->lag_fx);
    tmp = div_s(shl(1,sub(14,exp)),X_fx->lag_fx);/* 29-exp */
    diff_fx= shl(tmp,exp-10); /* Q19 */

    min_q = EVS_SW_MAX;
    move16();
    X_fx->a_fx[0]=0;
    move16();

    HalfLag_fx=s_min(shr(X_fx->lag_fx,1),X_fx->nH_fx);

    j=1;
    move16();
    FOR (i=1 ; i<=HalfLag_fx; i++)
    {
        Ltemp_fx=L_mult(diff_fx,i);  /*  Ltemp=i*diff, Q20 */
        freq_fx=round_fx(L_shl(Ltemp_fx,11)); /*  Q15                             */

        freq_fx=s_min(freq_fx,upper_cut_off_freq_fx); /*  0x4000 is 0.5 in Q15 */

        FOR ( ; j<m_fx; j++)
        {
            d1=sub(f_fx[j],freq_fx);

            IF (d1>=0)
            {
                d2=sub(freq_fx,f_fx[j-1]);
                Ltemp_fx=L_mac(L_mult(amp_fx[j],d2),amp_fx[j-1],d1); /*  Q29 */
                d2=sub(f_fx[j],f_fx[j-1]); /*  Q15 */
                /* Ltemp2_fx=invert_dp((Word40)d2, 4, &n,1); : Ltemp2=1/d2, Q(61-15-n) */
                exp = norm_s(d2);
                tmp = div_s(shl(1,sub(14,exp)),d2);/* 29-exp */
                /* L_tmp = L_shr(tmp,8); */
                Ltemp2_fx = L_shl(tmp,16);
                n = add(exp,16);

                d1h=extract_h(Ltemp_fx);
                d1l=extract_l(Ltemp_fx);
                d2h=extract_h(Ltemp2_fx);
                d2l=extract_l(Ltemp2_fx);
                Ltemp_fx=(Word32)L_mult0(d1h,d2l);
                Lacc_fx=L_mac0((Word32)Ltemp_fx,d2h,d1l);
                Ltemp_fx=L_add((Word32)L_shr(Lacc_fx,15),L_mult(d1h,d2h)); /*  46-n+29-31 */
                d2h=norm_l(Ltemp_fx); /*  d2h is 0 IF Ltemp=0 */
                d2h=(Ltemp_fx==0)?31:d2h; /*  make sure IF Ltemp=0, it gets the largest Q */
                X_fx->a_fx[i]=round_fx(L_shl(Ltemp_fx,d2h)); /*  Q(28-n+d2h) */
                q[i]=add(sub(28,n),d2h);
                min_q=s_min(min_q,q[i]);

                BREAK ;
            }
        }
    }
    /*  block normalize a[i] */
    FOR (i=1; i<=HalfLag_fx; i++)
    {
        X_fx->a_fx[i]=shl(X_fx->a_fx[i],sub(min_q,q[i]));
        move16();
    }

    X_fx->Q=min_q;
}

/*===================================================================*/
/* FUNCTION      :  LPCPowSpect_fx ()                                */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Compute LPC power spectrum                       */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16 []) freq :  ERB frequency bounds, Q15                 */
/*   _ (Word16 []) LPC :  LPC coefficients, Q12                      */
/*   _ (Word16)  Nf:  number of ERB bins, Q0                         */
/*   _ (Word16) Np :  order of LPC, Q0                               */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) out : LPC power spectrum, Q7                      */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX                                                  */
/*===================================================================*/
/* NOTE: Frequency is normalized by 12800, i.e. 1=12800Hz            */
/*===================================================================*/
void LPCPowSpect_fx(Word16 *freq, Word16 Nf, Word16 *LPC, Word16 Np,
                    Word16 *out)
{
    Word16 i, k;
    Word16 w; /*  Q9 */
    Word16 t1, dt;
    /*Word16 t2; */
    Word16 dh, dl;
    Word32 Re, Im;  /*  Q27 */
    Word32 Ltemp, Lw;
    Word32 Lacc;
    Word16 tmp,exp;

    FOR (k=0; k<Nf; k++)
    {

        Re = L_add(0x8000000, 0);   /*  Re=1.0, Q27 */
        Im = L_deposit_l(0);
        Lw=L_deposit_l(freq[k]);/*  Q15 */
        FOR (i=0; i<Np; i++)
        {
            Ltemp=L_shl(Lw,10); /*  Ltemp in Q25 */
            w=extract_h(Ltemp); /*  w in Q9 */
            dl=extract_l(Ltemp); /*  dl has 6 bits left-over     */
            w = s_and(w,511);
            t1=cos_table[w];
            /* t2=cos_table[s_and(add(w,1),511)]; */
            /*dt=sub(t2,t1); */ /*  dt=t2-t1, Q15 */
            dt = cos_diff_table[w];

            IF(dl<0)
            {
                Ltemp = L_shl(L_add(65536,dl),14);/*  */
                Ltemp = Mult_32_16(Ltemp,dt);
                Ltemp = L_shl(Ltemp,1);
            }
            ELSE
            {
                Ltemp=(Word32)L_mult0(dt,dl); /*  Ltemp in Q31 */
            }

            t1=add(t1,(Word16)L_shr(Ltemp,16)); /*  t1 is interpolated cos(w) */
            Ltemp=L_shr(L_mult(LPC[i],t1),1); /*  Ltemp in Q27 */
            Re=L_add(Re, Ltemp); /*  Re=1-sum(LPC[i]*cos(Lw)); */
            Ltemp=L_add(Lw,0x6000); /*  add 0.75, which is 3pi/2 to convert sin to cos */
            Ltemp=L_shl(Ltemp,10); /*  Q25 */
            w=extract_h(Ltemp); /*  w is equivalent cos index */
            dl=extract_l(Ltemp); /*  dl is 6 bit left-over for interpolation */
            w = s_and(w,511);
            t1=cos_table[w];
            /*t2=cos_table[s_and(add(w,1),511)]; */
            /*dt=sub(t2,t1); */ /*  dt=t2-t1, Q15 */
            dt = cos_diff_table[w];

            IF(dl<0)
            {
                Ltemp = L_shl(L_add(65536,dl),14);/*  */
                Ltemp = Mult_32_16(Ltemp,dt);
                Ltemp = L_shl(Ltemp,1);
            }
            ELSE
            {
                Ltemp=(Word32)L_mult0(dt,dl); /*  Ltemp in Q31 */
            }

            t1=add(t1,(Word16)L_shr(Ltemp,16)); /*  t1 is interpolated cos(w) */
            Ltemp=L_shr(L_mult(LPC[i],t1),1); /*  Ltemp in Q27 */
            Im=L_sub(Im,Ltemp); /*  Im=sum(LPC[i]*sin(Lw)) */
            Lw=L_add(Lw,freq[k]); /*  Lw=(i+1)*freq[k] */
        }
        /* If necessary, we can block-normalize Re and Im to improve precision */
        dh=extract_h(Re);
        dl=extract_l(Re);

        IF(dl<0)
        {
            Ltemp = L_shl(L_add(65536,dl),14);/*  */
            Ltemp = Mult_32_16(Ltemp,dh);
            Lacc = L_shl(Ltemp,1);
        }
        ELSE
        Lacc=L_mult0(dh,dl);

        Lacc=L_add(L_shr(Lacc,15),L_shr(L_mult(dh,dh),1)); /*  Lacc=Re*Re */
        dh=extract_h(Im);
        dl=extract_l(Im);

        IF(dl<0)
        {
            Ltemp = L_shl(L_add(65536,dl),14);/*  */
            Ltemp = Mult_32_16(Ltemp,dh);
            Ltemp = L_shl(Ltemp,1);
        }
        ELSE
        Ltemp=(Word32)L_mult0(dh,dl);

        Lacc=L_add(Lacc,L_shr(Ltemp,15));
        Lacc=L_add(Lacc,L_shr(L_mult(dh,dh),1)); /*  Lacc=Re^2+Im^2, Q22        */

        exp = norm_l(Lacc);
        tmp = round_fx(L_shl(Lacc,exp));
        exp = sub(sub(30,exp),22);

        /* tmp may potentially become negative, when Lacc is a very large value */
        IF(tmp > 0)
        {
            tmp = div_s(16384, tmp); /* 15+exp1 */
        }
        ELSE
        {
            tmp = 0;
            move16();
        }
        Ltemp = L_deposit_h(tmp);
        out[k] = round_fx(L_shl(Ltemp,negate(add(exp,8))));

        /* out[k] = shl(tmp,-exp-8); in Q7 */

    }
}
/*===================================================================*/
/* FUNCTION      :  erb_diff_fx ()                                   */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Quantize erb amplitude for QPPP                  */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) pl :  previous pitch lag, Q0                         */
/*   _ (Word16) l :   current pitch lag, Q0                          */
/*   _ (Word16 []) prev_erb : Previous erb amplitude, Q13            */
/*   _ (Word16 []) curr_erb : Current erb amplitude, Q13             */
/*   _ (Word16 []) curr_lsp : LSP coefficients, Q12                  */
/*   _ (Word16 [])  num_erb : Number of ERBs  , Q0                   */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) index: quantized differential erb index           */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX                                                  */
/*===================================================================*/
static
void erb_diff_fx(
    const Word16 *prev_erb,            /* i  : previous ERB              */
    Word16   pl,                       /* i  : previous lag              */
    const Word16 *curr_erb,            /* i  : current ERB               */
    Word16   l,                        /* i  : current lag               */
    const Word16 *curr_lsp,            /* i  : current LSP coefficients  */
    Word16 *index,                     /* 0  : ERB index                 */
    Word16 num_erb                     /* i  : Number of ERBs            */
)
{
    Word16 i;
    Word16 pslot[NUM_ERB_WB], cslot[NUM_ERB_WB];
    Word16 tmp, t_prev_erb[NUM_ERB_WB], LPC[M+1], mfreq[NUM_ERB_WB], PowSpect[NUM_ERB_WB], dif_erb[NUM_ERB_WB] ;
    const Word16 *AmpCB1_fx = NULL;

    IF (sub(num_erb,NUM_ERB_NB)==0)
    {
        AmpCB1_fx=AmpCB1_NB_fx;
        move16();
    }
    ELSE IF (sub(num_erb,NUM_ERB_WB)==0)
    {
        AmpCB1_fx=AmpCB1_WB_fx;
        move16();

    }
    erb_slot_fx (l,cslot,mfreq,num_erb);/* cslot in Qo and mfreq in Q15 */
    erb_slot_fx (pl,pslot,t_prev_erb,num_erb);

    FOR (i=0; i<M+1; i++)
    {
        LPC[i]=mult_r(curr_lsp[i],pwf78_fx[i]);
        move16();
    }

    LPCPowSpect_fx (mfreq, num_erb, LPC, M+1, PowSpect);/* Powspect in Q7 */

    FOR (i=0; i<num_erb; i++)
    {
        if (cslot[i]==0)
        {
            PowSpect[i]=0;
            move16();
        }
    }
    FOR(i=0; i<num_erb; i++)
    {
        t_prev_erb[i] = prev_erb[i] ;
        move16();
    }
    IF(sub(pl,l)>0)
    {
        tmp = t_prev_erb[0] ;
        move16();
        FOR (i=0; i<num_erb; i++)
        {
            IF (pslot[i]!=0)
            {
                tmp = t_prev_erb[i] ;
                move16();
            }
            ELSE
            {
                t_prev_erb[i] = tmp ;
                move16();
            }
        }
    }
    ELSE IF (sub(l,pl)>0)
    {
        tmp = t_prev_erb[num_erb-1] ;
        move16();

        FOR(i=sub(num_erb,1); i>=0; i--)
        {
            IF (pslot[i]!=0)
            {
                tmp = t_prev_erb[i];
                move16();
            }
            ELSE
            {
                t_prev_erb[i] = tmp ;
                move16();
            }
        }
    }
    FOR(i=0; i<num_erb; i++)
    {
        dif_erb[i] = sub(curr_erb[i],t_prev_erb[i]);
        move16();
    }

    /* First Band Amplitude Search */
    index[0] = erb_diff_search_fx(t_prev_erb, curr_erb, dif_erb,
                                  PowSpect, AmpCB1_fx,
                                  ERB_CBSIZE1, 10, 1) ;
    move16();
    IF (sub(num_erb,NUM_ERB_NB)==0)
    {
        /* Second Band Amplitude Search */
        index[1] = erb_diff_search_fx(t_prev_erb, curr_erb, dif_erb,
                                      PowSpect, AmpCB2_NB_fx,
                                      ERB_CBSIZE2, 9, 11) ;
        move16();
    }
    ELSE IF (sub(num_erb,NUM_ERB_WB)==0)
    {
        /* Second Band Amplitude Search */
        index[1] = erb_diff_search_fx(t_prev_erb, curr_erb, dif_erb,
                                      PowSpect, AmpCB2_WB_fx,
                                      ERB_CBSIZE2, 11, 11) ;
        move16();
    }

}

/*===================================================================*/
/* FUNCTION      :  erb_add_fx ()                                    */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Reconstruct current erb amplitude for QPPP       */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) pl_fx :  previous pitch lag, Q0                      */
/*   _ (Word16) l_fx :   current pitch lag, Q0                       */
/*   _ (Word16 []) prev_erb_fx : Previous erb amplitude, Q13         */
/*   _ (Word16 []) index_fx: quantized differential erb index        */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) curr_erb_fx : Current erb amplitude, Q13          */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void erb_add_fx(
    Word16 *curr_erb_fx,            /* i/o:  current ERB    */
    Word16   l_fx,                  /* i  :  current lag    */
    const Word16 *prev_erb_fx,      /* i  :  previous ERB   */
    Word16   pl_fx,                 /* i  :  previous lag   */
    const Word16   *index_fx,       /* i  :  ERB index      */
    Word16 num_erb_fx               /* i  :  number of ERBs */
)
{
    Word16 i ;
    Word16 pslot_fx[NUM_ERB_WB],cslot_fx[NUM_ERB_WB],t_prev_erb_fx[NUM_ERB_WB];
    Word16 tmp_fx,tmp2_fx,tmp_loop;
    const Word16 *AmpCB1_fx = NULL; /*move16(); */

    IF (sub(num_erb_fx,NUM_ERB_NB)==0)
    {
        AmpCB1_fx=AmpCB1_NB_fx;
        move16();
    }
    ELSE IF (sub(num_erb_fx,NUM_ERB_WB)==0)
    {
        AmpCB1_fx=AmpCB1_WB_fx;
        move16();
    }

    erb_slot_fx(l_fx,cslot_fx,t_prev_erb_fx,num_erb_fx);
    erb_slot_fx(pl_fx,pslot_fx,t_prev_erb_fx,num_erb_fx);

    FOR (i=0; i<num_erb_fx; i++)
    {
        t_prev_erb_fx[i] = prev_erb_fx[i] ;
        move16(); /* Q13 */
    }


    IF (sub(pl_fx,l_fx)>0)
    {
        tmp_fx = t_prev_erb_fx[0];
        move16(); /* Q13  */
        FOR (i=0; i<num_erb_fx; i++)
        {

            IF (pslot_fx[i] != 0)
            {
                tmp_fx = t_prev_erb_fx[i];
                move16(); /* Q13 */
            }
            ELSE
            {
                t_prev_erb_fx[i] = tmp_fx;
                move16(); /* Q13 */
            }
        }
    }
    ELSE IF (sub(l_fx,pl_fx)>0)
    {
        tmp_fx = t_prev_erb_fx[sub(num_erb_fx,1)]; /* Q13 */
        FOR (i=sub(num_erb_fx,1); i>=0; i--)
        {

            IF (pslot_fx[i] != 0)
            {
                tmp_fx = t_prev_erb_fx[i];
                move16(); /* Q13 */
            }
            ELSE
            {
                t_prev_erb_fx[i] = tmp_fx;
                move16(); /* Q13 */
            }
        }
    }

    tmp_fx = add(shl(index_fx[0],3),shl(index_fx[0],1)); /*  tmp_fx=10*index_fx[0] */
    FOR (i=1; i<11; i++)
    {

        IF (cslot_fx[i]!=0)
        {
            curr_erb_fx[i] = add(AmpCB1_fx[sub(add(tmp_fx,i),1)],t_prev_erb_fx[i]);
            move16();/*  Q13+Q13=Q13 */
            curr_erb_fx[i] = s_max(0, curr_erb_fx[i]);
            move16();
        }
        ELSE
        curr_erb_fx[i] = 0;
        move16();
    }

    tmp_fx = add(shl(index_fx[1],3),index_fx[1]); /*  tmp=9*index[1] */
    tmp2_fx = mult(shl(index_fx[1],6),5632);/* temp=11*index_fx[1] */
    tmp_loop = sub(num_erb_fx,2);
    FOR (i=11; i<tmp_loop; i++)
    {

        IF (cslot_fx[i]!=0)
        {
            IF (sub(num_erb_fx,NUM_ERB_NB)==0)
            {
                curr_erb_fx[i] = add(AmpCB2_NB_fx[sub(add(tmp_fx,i),11)],t_prev_erb_fx[i]) ;/* Q13+Q13=Q13 */
                curr_erb_fx[i] = s_max(0, curr_erb_fx[i]);
                move16();
            }
            ELSE IF (sub(num_erb_fx,NUM_ERB_WB)==0)
            {
                curr_erb_fx[i] = add(AmpCB2_WB_fx[sub(add(tmp2_fx,i),11)],t_prev_erb_fx[i]) ; /* Q13 */
                curr_erb_fx[i] = s_max(0, curr_erb_fx[i]);
                move16();
            }
        }
        ELSE
        curr_erb_fx[i] = 0;
        move16();
    }
}

/*===================================================================*/
/* FUNCTION      :  Word16 DTFS_quant_cw_fx ()                       */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Quantize QPPP prototype                          */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) pl: previous lag                                     */
/*   _ (Word16 []) curr_lpc_fx: LPC coefficients, Q12                */
/*   _ (Word16 []) sin_tab: sine table based on lag, Q15             */
/*   _ (Word16 []) cos_tab: cosine table based on lag, Q15           */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16) POWER_IDX: Power index                               */
/*   _ (Word16[]) AMP_IDX: Amplitude indices                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (struct DTFS_fx) X_fx :  prototype in polar domain            */
/*                (Word16) lag_fx: length of prototype in time domain*/
/*                (Word16 []) a: amplitude of harmonics, normalized  */
/*                (Word16) Q: norm factor of a                       */
/*   _ (Word16[]) lasterb_fx: ERB history for differential           */
/*                quantization, Q13                                  */
/*   _ (Word16) Lgain_fx: low band power history, log domain, Q11    */
/*   _ (Word16) Hgain_fx: high band power history, log domain, Q11   */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*   _ (Word16) flag: flag indicating success/failure (TRUE/FALSE)   */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX                                                  */
/*===================================================================*/
/* NOTE: Frequencies is normalized by 12800, i.e. 1=12800Hz          */
/*===================================================================*/
Word16 DTFS_quant_cw_fx(
    DTFS_STRUCTURE_FX *X_fx,    /* i/o: DTFS unquant inp, quant out  */
    Word16  pl,                 /* i  : Previous lag                 */
    const Word16 *curr_lpc_fx,        /* i  : LPC                          */
    Word16 *POWER_IDX,          /* o  : Power index                  */
    Word16 *AMP_IDX,            /* o  : Amplitude index              */
    Word16 *lastLgainE_fx,      /* i/o: last frame lowband gain      */
    Word16 *lastHgainE_fx,      /* i/o: last frame highband gain     */
    Word16 *lasterbE_fx,        /* i/o: last frame ERB vector        */
    Word16 *sin_tab,
    Word16 *cos_tab
)

{
    Word16 num_erb = 0;
    const Word16 *PowerCB_fx =NULL;
    Word16 tmp, w[2], target[2],j,slot[NUM_ERB_WB],flag;
    Word16 n,d1,d2,exp;
    Word32 minerror,Ltemp,logLag_fx ,L_tmp;
    Word16 erb_uq[NUM_ERB_WB],Qh,Ql;
    /* Word40 Lacc_40; */
    Word32 Lacc;
    Word16 mfreq[NUM_ERB_WB] ;
    Word16 Q;

    Word16 curr_erb_fx[NUM_ERB_WB] ;


    /* upper_cute_off_freq are normalized to 12800 */

    IF (sub(X_fx->upper_cut_off_freq_fx,0x2800)==0)/* 4000 hz normalized to 12800 in Q15 */
    {
        num_erb=NUM_ERB_NB;
        move16();
        PowerCB_fx=PowerCB_NB_fx;
        move16();
    }
    ELSE IF (sub(X_fx->upper_cut_off_freq_fx,0x4000)==0)/* 6400 hz normalized to 12800 in Q15 */
    {
        num_erb=NUM_ERB_WB;
        move16();
        PowerCB_fx=PowerCB_WB_fx;
        move16();
    }

    /*  Get weighting and target */
    quant_target_fx(X_fx,curr_lpc_fx, w, target, sin_tab, cos_tab);

    /* Power Quantization in log domain */
    target[0]=sub(target[0],*lastLgainE_fx);
    move16();
    target[1]=sub(target[1],*lastHgainE_fx);
    move16();

    minerror = L_add(EVS_LW_MAX, 0);
    *POWER_IDX = 0;
    move16();

    j=0;
    move16();
    FOR (n=0; n<P_CBSIZE*2; n+=2)
    {
        /*     n=shl(j,1);  n=offset to current codebook entry */
        d1=sub(target[0],PowerCB_fx[n]);
        d2=sub(target[1],PowerCB_fx[n+1]);
        Ltemp=L_mult(w[0],abs_s(d1));
        Ltemp=L_mac(Ltemp,w[1],abs_s(d2)); /*  Ltemp=error */

        test();
        IF (d1>=0 && d2>=0)
        {
            Ltemp= Mult_32_16(Ltemp,0x6666); /*  *=0.8 */
        }
        IF (L_sub(Ltemp, minerror)<0)
        {
            minerror = L_add(Ltemp, 0);
            *POWER_IDX = j ;
            move16();
        }
        j = add(j,1);
    }
    DTFS_to_erb_fx (*X_fx,curr_erb_fx) ;

    FOR(j=0; j<num_erb; j++)
    {
        erb_uq[j] = curr_erb_fx[j];
        move16();
    }
    erb_slot_fx (X_fx->lag_fx, slot, mfreq,num_erb) ;
    /* Amplitude Quantization */


    erb_diff_fx(lasterbE_fx,pl,curr_erb_fx,X_fx->lag_fx,curr_lpc_fx,AMP_IDX,num_erb);


    /*  Dequantization of prototype */
    /*  PORTING: Removing the references */
    /* DTFS_dequant_cw_fx(pl, *POWER_IDX, AMP_IDX,lastLgainE_fx,lastHgainE_fx, lasterbE_fx,X_fx,num_erb,curr_erb_fx); */

    /* Determine IF the amplitude quantization is good enough */
    erb_add_fx(curr_erb_fx,X_fx->lag_fx,lasterbE_fx,pl,AMP_IDX,num_erb);

    curr_erb_fx[0] = mult_r(curr_erb_fx[1],9830);
    move16(); /* 0.3 inQ15 leaves curr_erb in Q13 */
    curr_erb_fx[sub(num_erb,2)] = mult_r(curr_erb_fx[sub(num_erb,3)],9830);/* Q13 */

    curr_erb_fx[sub(num_erb,1)] = 0;
    move16();
    flag=1;
    move16();

    Ltemp = L_deposit_l(0);
    n=0;
    move16();
    FOR (j=1; j<10; j++)
    {
        IF (slot[j]!=0)
        {
            Ltemp=L_add(Ltemp,abs_s(sub(erb_uq[j],curr_erb_fx[j]))); /*  Q13 */
            n=add(n,1); /*  n++ */
        }
    }

    exp = norm_s(n);
    tmp = div_s(shl(1,sub(14,exp)),n);/* 29 - exp */
    Lacc = L_shl( Mult_32_16(Ltemp , tmp), exp +4);

    tmp=round_fx(Lacc); /*  tmp in Q15 */

    test();
    if (sub(tmp,0x3C29)>0 && add(target[0],819)>0)
    {
        flag = 0 ; /* Bumping up */ move16();
    }

    /* mfreq normalized (2.56) in Q15 */
    DTFS_erb_inv_fx(curr_erb_fx, slot, mfreq, X_fx,num_erb) ;


    /* Back up the lasterbD memory after power normalization */
    DTFS_setEngyHarm_fx(236,2828,0,2828,1,0,&Ql,X_fx);
    DTFS_setEngyHarm_fx(2828,X_fx->upper_cut_off_freq_of_interest_fx,2828,X_fx->upper_cut_off_freq_fx,1,0,&Qh,X_fx);

    /* Need to unify the Q factors of both bands */
    X_fx->Q=s_min(Ql,Qh); /*  set Q factor to be the smaller one */
    n=sub(Ql,Qh); /*  compare band Q factors */


    /* This logic adjusts difference between Q formats of both bands */

    IF (n<0)
    rshiftHarmBand_fx(X_fx,2828, X_fx->upper_cut_off_freq_fx,n);
    ELSE IF (n>0)
    rshiftHarmBand_fx(X_fx,0, 2828, sub(Qh,Ql));

    tmp=shl(*POWER_IDX,1); /*  tmp=2*POWER_IDX */
    *lastLgainE_fx =add(*lastLgainE_fx, PowerCB_fx[tmp]) ; /*  Q11 */
    *lastHgainE_fx =add(*lastHgainE_fx, PowerCB_fx[tmp+1]); /*  Q11 */

    Ltemp=log10_fx(X_fx->lag_fx); /*  Ltemp=10*log10(lag), Q23 */
    logLag_fx= Mult_32_16(Ltemp,0x6666); /*  logLag=log10(lag), Q26 */

    Ltemp=L_sub(L_shr(L_deposit_h(*lastLgainE_fx),1),logLag_fx); /*  Ltemp=Lgain-log10(lag), Q26 */

    L_tmp=pow_10(Ltemp,&Q); /*  Lacc=10^Lgain/lag, Q15 */
    n=norm_l(L_tmp);
    Ltemp=(Word32)L_shl(L_tmp,n); /*  Ltemp in Q(15+n) */


    DTFS_setEngyHarm_fx(236,2828,0,2828,Ltemp, add(Q,n),&Ql,X_fx);

    Ltemp=L_sub(L_shr(L_deposit_h(*lastHgainE_fx),1),logLag_fx); /*  Ltemp=Hgain-log10(lag), Q26 */

    /* Ltemp = L_shr(Ltemp,1); */
    L_tmp = pow_10(Ltemp,&Q); /*  Lacc=10^Lgain/lag, Q15 */
    n=norm_l(L_tmp);
    Ltemp=(Word32)L_shl(L_tmp,n); /*  Ltemp in Q(15+n) */

    DTFS_setEngyHarm_fx(2828,X_fx->upper_cut_off_freq_of_interest_fx,2828,X_fx->upper_cut_off_freq_fx,Ltemp,add(Q,n),&Qh,X_fx);
    /* Need to unify the Q factors of both bands */
    X_fx->Q=s_min(Ql,Qh); /*  set Q factor to be the smaller one */
    n=sub(Ql,Qh); /*  compare band Q factors */

    IF (n<0)
    rshiftHarmBand_fx(X_fx,2828, X_fx->upper_cut_off_freq_fx,n);
    ELSE IF (n>0)
    rshiftHarmBand_fx(X_fx,0, 2828, sub(Qh,Ql));

    return flag;
}
/*===================================================================*/
/* FUNCTION      :  struct quant_target_fx()                          */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Get weighting and target for power quantization  */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16 []) curr_lpc: LPC coefficients, Q12                   */
/*   _ (Word16 []) sin_tab: sine table based on lag, Q15             */
/*   _ (Word16 []) cos_tab: cosine table based on lag, Q15           */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16[]) w: Weighting for power quantization, Q15           */
/*   _ (Word16[]) target: Power of 2 bands for quantization, Q11     */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (struct DTFS_fx) X :  prototype in polar domain                      */
/*                (Word16) lag: length of prototype in time domain   */
/*                (Word16 []) a: amplitude of harmonics, normalized  */
/*                (Word16) Q: norm factor of a                       */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX                                                  */
/*===================================================================*/
/* NOTE: This function is used by quant_cw_fx and quant_cw_memless_fx*/
/*===================================================================*/
void quant_target_fx( DTFS_STRUCTURE_FX *X_fx,const Word16 *curr_lpc, Word16 *w, Word16 *target,
                      Word16 *sin_tab, Word16 *cos_tab)
{
    Word16 tmp, n,exp;
    Word16 Ql, Qh;
    Word32 Ltemp1, Ltemp, Ltemp2, logLag, Ltemp3,Lacc;

    tmp=sub(shl(X_fx->Q,1),13); /*  tmp=2Q-13, Q factor of getSpEngy... output */
    Ltemp3=L_shl(L_mult(tmp,24660),9); /*  Ltemp3=(2Q-13)*10log10(2), Q23, 24660=10log10(2) in Q13 */

    Ltemp1 = getSpEngyFromResAmp_fx(X_fx,0, 2828,curr_lpc, sin_tab, cos_tab);
    Ltemp1=log10_fx(Ltemp1);

    /*  subtract 10log10(2)*(2Q-13), Q23 */
    Ltemp1=L_sub(Ltemp1,Ltemp3);
    Ltemp1=L_max(0,Ltemp1);
    Ltemp2 = getSpEngyFromResAmp_fx(X_fx,2828,X_fx->upper_cut_off_freq_fx,curr_lpc, sin_tab, cos_tab);
    Ltemp2=log10_fx(Ltemp2 ); /*  Ltemp1=10log10(eng_hb), Q23, need to adjust for Q factor of energy (2Q-13) */
    Ltemp2=L_sub(Ltemp2,Ltemp3); /*  Ltemp2 in Q23 */

    Ltemp2=L_max(0,Ltemp2);

    /* Getting the Speech Domain Energy LOG Ratio    */

    Lacc = L_add(Ltemp1,Ltemp2);
    exp = norm_l(Lacc);
    tmp = round_fx(L_shl(Lacc,exp));
    exp = sub(sub(30,exp),23);
    IF(tmp)
    tmp = div_s(16384,tmp); /* 15+exp1 */
    ELSE
    tmp = 0;
    Ltemp = L_deposit_h(tmp);

    tmp=round_fx(Ltemp); /*  tmp in Q(22-n) */
    Ltemp1= Mult_32_16(Ltemp1,tmp); /*  Q(30-n) */
    n=sub(8,exp);
    w[0]=round_fx(L_shl(Ltemp1,n)); /*  w[0] in Q15 */
    Ltemp2= Mult_32_16(Ltemp2,tmp);
    w[1]=round_fx(L_shl(Ltemp2,n)); /*  w[1] in Q15 */

    logLag=log10_fx(X_fx->lag_fx); /*  logLag=10*log10(lag), Q23 */
    Ltemp3=L_shl(L_mult(shl(X_fx->Q,1),24660),9); /*  Ltemp3=2Q*10log10(2), Q23 */
    /* Process low band */
    Ltemp=DTFS_setEngyHarm_fx(236, 2828, 0, 2828, 1, 0, &Ql,X_fx); /*  Ql is norm factor of low band a[], Ltemp is energy in 2Q */
    /* Compensate for Q factor of energy to get log10(lag*eng)  */
    Ltemp=log10_fx(Ltemp); /*  Ltemp=10log10(eng), Q23 */
    Ltemp=L_add(L_sub(Ltemp,Ltemp3),logLag);  /*  Ltemp=10*log10(lag*eng), Q23 */

    target[0]=round_fx(L_shl(Mult_32_16(Ltemp,0x6666),1)); /*  Q11 */

    /* Process high band */
    Ltemp=DTFS_setEngyHarm_fx(2828, X_fx->upper_cut_off_freq_of_interest_fx, 2828, X_fx->upper_cut_off_freq_fx, 1, 0, &Qh,X_fx);
    Ltemp=log10_fx(Ltemp);
    Ltemp=L_add(L_sub(Ltemp,Ltemp3),logLag); /*  Ltemp=10*log10(lag*eng), Q23 */
    target[1]=round_fx(L_shl(Mult_32_16(Ltemp,0x6666),1)); /*  Q11 */

    /* Need to unify the Q factors of both bands */
    X_fx->Q=s_min(Ql,Qh); /*  set Q factor to be the smaller one */
    n=sub(Ql,Qh); /*  compare band Q factors */

    IF (n<0) rshiftHarmBand_fx(X_fx,2828, X_fx->upper_cut_off_freq_fx,n);
    ELSE IF (n>0) rshiftHarmBand_fx(X_fx,0, 2828, sub(Qh,Ql));
}
/*===================================================================*/
/* FUNCTION      :  struct DTFS_fx::dequant_cw_fx ()                 */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Dequantize QPPP prototype                        */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) pl: previous lag                                     */
/*   _ (Word16) p_idx: Power index                                   */
/*   _ (Word16[]) a_idx: Amplitude indices, 2 words                  */
/*   _ (struct DTFS_fx) X :  prototype in polar domain               */
/*                (Word16) lag: length of prototype                  */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (struct DTFS_fx) X :  prototype in polar domain               */
/*                (Word16) lag: length of prototype in time domain   */
/*                (Word16 []) a: amplitude of harmonics, normalized  */
/*                (Word16) Q: norm factor of a                       */
/*   _ (Word16[]) curr_erb: Quantized current ERB, Q13               */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (Word16[]) lasterb: ERB history for differential              */
/*                quantization, Q13                                  */
/*   _ (Word16) Lgain: low band power history, log domain, Q11       */
/*   _ (Word16) Hgain: high band power history, log domain, Q11      */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void DTFS_dequant_cw_fx(
    Word16   pl_fx,                  /* i  : Previous lag                 */
    Word16   POWER_IDX_fx,           /* i  : POWER index                  */
    const Word16   *AMP_IDX_fx,      /* i  : Amp Shape index              */
    Word16 *lastLgainD_fx,           /* i/o: low band last gain           */
    Word16 *lastHgainD_fx,           /* i/o: high band last gain          */
    Word16 *lasterbD_fx,             /* i/o: last frame ERB vector        */
    DTFS_STRUCTURE_FX *X_fx,         /* o  : DTFS structure dequantized   */
    Word16 num_erb_fx
)

{
    Word16 tmp_fx, mfreq_fx[NUM_ERB_WB], curr_erb_fx[NUM_ERB_WB];
    const Word16 *PowerCB_fx=NULL;
    Word16 slot_fx[NUM_ERB_WB];
    Word16 Ql, Qh, n;
    Word32 Ltemp_fx,logLag_fx;
    Word16 exp,frac,exp1;
    Word32 L_tmp,L_temp;


    IF (num_erb_fx==NUM_ERB_NB)
    {
        PowerCB_fx=PowerCB_NB_fx;
        move16();
    }
    ELSE IF (num_erb_fx==NUM_ERB_WB)
    {
        PowerCB_fx=PowerCB_WB_fx;
        move16();
    }

    /* Amplitude Dequantization */

    erb_add_fx(curr_erb_fx,X_fx->lag_fx,lasterbD_fx,pl_fx,AMP_IDX_fx,num_erb_fx);

    curr_erb_fx[0] = mult_r(curr_erb_fx[1],9830);/* 0.3 inQ15 leaves curr_erb in Q13 */
    curr_erb_fx[sub(num_erb_fx,2)] = mult_r(curr_erb_fx[sub(num_erb_fx,3)],9830);/* Q13 */

    move16();

    curr_erb_fx[sub(num_erb_fx,1)] = 0;

    erb_slot_fx(X_fx->lag_fx,slot_fx,mfreq_fx,num_erb_fx);

    /* mfreq normalized (2.56) in Q15 */
    DTFS_erb_inv_fx(curr_erb_fx, slot_fx, mfreq_fx, X_fx,num_erb_fx) ;


    /* Back up the lasterbD memory after power normalization */

    DTFS_setEngyHarm_fx(236,2828,0,2828,1,0,&Ql,X_fx);
    DTFS_setEngyHarm_fx(2828,X_fx->upper_cut_off_freq_of_interest_fx,2828,X_fx->upper_cut_off_freq_fx,1,0,&Qh,X_fx);

    /* Need to unify the Q factors of both bands */
    X_fx->Q=s_min(Ql,Qh); /*  set Q factor to be the smaller one */
    n=sub(Ql,Qh); /*  compare band Q factors */




    /* This logic adjusts difference between Q formats of both bands */
    IF (n<0)
    rshiftHarmBand_fx(X_fx,2828, X_fx->upper_cut_off_freq_fx,n);
    ELSE IF (n>0)
    rshiftHarmBand_fx(X_fx,0, 2828, sub(Qh,Ql));

    DTFS_to_erb_fx(*X_fx,lasterbD_fx);


    /* Power Dequantization */

    tmp_fx=shl(POWER_IDX_fx,1); /*  tmp=2*POWER_IDX */
    *lastLgainD_fx =add(*lastLgainD_fx, PowerCB_fx[tmp_fx]) ; /*  Q11 */
    *lastHgainD_fx =add(*lastHgainD_fx, PowerCB_fx[tmp_fx+1]); /*  Q11 */

    L_tmp = L_deposit_h(X_fx->lag_fx); /* Q16 */
    exp = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp,exp);
    frac = Log2_norm_lc(L_tmp);
    exp = sub(30,add(exp,16));
    L_tmp = Mpy_32_16(exp,frac,12330);/* Q13 */ /* 10*log10(2) in Q12*/
    Ltemp_fx = L_shl(L_tmp, 10);/* Q23 */


    logLag_fx= Mult_32_16(Ltemp_fx,0x6666); /*  logLag=log10(lag), Q26 */

    Ltemp_fx=L_sub(L_shr(L_deposit_h(*lastLgainD_fx),1),logLag_fx); /*  Ltemp=Lgain-log10(lag), Q26 */

    /* Lacc_fx=dsp_pow10(Ltemp_fx); : Lacc=10^Lgain/lag, Q15 */

    L_tmp = Mult_32_16(Ltemp_fx, 27213); /* 3.321928 in Q13 */
    L_tmp = L_shr(L_tmp,8);            /* Q16     */
    frac = L_Extract_lc(L_tmp, &exp1); /* Extract exponent  */
    L_tmp =Pow2(14, frac);
    exp1 = sub(exp1,14);
    L_temp = L_shl(L_tmp, add(exp1,15) ); /* Q15 */

    n=norm_l(L_temp);
    Ltemp_fx=L_shl(L_temp,n); /*  Ltemp in Q(15+n) */

    DTFS_setEngyHarm_fx(236,2828,0,2828,Ltemp_fx, add(15,n),&Ql,X_fx);

    Ltemp_fx=L_sub(L_shr(L_deposit_h(*lastHgainD_fx),1),logLag_fx); /*  Ltemp=Hgain-log10(lag), Q26 */
    /* Lacc_fx=dsp_pow10(Ltemp_fx); : Lacc=10^Hgain/lag, Q15 */
    L_tmp = Mult_32_16(Ltemp_fx, 27213); /* 3.321928 in Q13 */ /* Q24 */
    L_tmp = L_shr(L_tmp,8);            /* Q16     */
    frac = L_Extract_lc(L_tmp, &exp1); /* Extract exponent  */
    L_tmp =Pow2(14, frac);
    exp1 = sub(exp1,14);
    L_temp = L_shl(L_tmp,exp1 +15 ); /* Q15 */
    n=norm_l(L_temp);
    Ltemp_fx=L_shl(L_temp,n); /*  Ltemp in Q(15+n) */

    DTFS_setEngyHarm_fx(2828,X_fx->upper_cut_off_freq_of_interest_fx,2828,X_fx->upper_cut_off_freq_fx,Ltemp_fx,add(15,n),&Qh,X_fx);
    /* Need to unify the Q factors of both bands */
    X_fx->Q=s_min(Ql,Qh); /*  set Q factor to be the smaller one */
    n=sub(Ql,Qh); /*  compare band Q factors */



    IF (n<0)
    rshiftHarmBand_fx(X_fx,2828, X_fx->upper_cut_off_freq_fx,n);
    ELSE IF (n>0)
    rshiftHarmBand_fx(X_fx,0, 2828, sub(Qh,Ql));
}
/*==========================================================================*/
/* FUNCTION      :  WIsyn_fx ()                                             */
/*--------------------------------------------------------------------------*/
/* PURPOSE       :                                                          */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                       */
/*   _ (struct DTFS_STRUCTURE_FX) PREVCW_FX: a/b in PREVCW_FX.Q             */
/*   _ (struct DTFS_fx *) CURR_CW_DTFS_FX: a/b in CURR_CW_DTFS_FX->Q        */
/*   _ (Word16 *) curr_lpc_fx: lpc coefficients in Q12                      */
/*   _ (Word16 *) ph_offset_fx: in Q15, normalized by 2pi                   */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                       */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                       */
/*   _ (Word16 *) N:     length, Q0                                         */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/*   _ (Word16 *) out_fx: Q0                                                */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/* _ (struct DTFS_STRUCTURE_FX *) CURR_CW_DTFS_FX: a/b in CURR_CW_DTFS_FX->Q*/
/* _ (Word16 *) ph_offset_fx: in Q15, normalized by 2pi                     */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                               */
/*==========================================================================*/

void WIsyn_fx(
    DTFS_STRUCTURE_FX PREVCW_FX,
    DTFS_STRUCTURE_FX *CURR_CW_DTFS_FX,
    const Word16 *curr_lpc_fx,
    Word16 *ph_offset_fx,
    Word16 *out_fx,
    Word16 N,
    Word16 FR_flag,              /* i  : called for post-smoothing in FR                */
    Word16 * S_fx,
    Word16* C_fx,
    Word16 *pf_temp1,
    Word16 *pf_temp2,
    Word16 *pf_temp,
    Word16 *pf_n2
)
{
    Word16 i;
    UWord16 I=1, flag=0;
    /* Word16 *phase_fx = (Word16*)malloc(sizeof(Word16) * (2*N));         new operator used size 2*N */
    Word32 phase_fx[WI_FX_phase_fx];
    Word16 alignment_fx;
    Word16 tmp_fx, temp;
    Word32 L_temp;
    Word16 exp,tmp;
    Word32 L_tmp;
    /* prev_lsp_fx; */


    DTFS_STRUCTURE_FX *CURRCW_DTFS_FX=DTFS_new_fx();


    IF (sub(PREVCW_FX.Q,CURR_CW_DTFS_FX->Q) < 0)
    {
        temp = sub(CURR_CW_DTFS_FX->Q, PREVCW_FX.Q);
        tmp = s_min(shr(CURR_CW_DTFS_FX->lag_fx,1),CURR_CW_DTFS_FX->nH_fx);
        FOR (i=0; i<=tmp; i++)
        {
            CURR_CW_DTFS_FX->a_fx[i] = shr(CURR_CW_DTFS_FX->a_fx[i], temp);
            move16();
            CURR_CW_DTFS_FX->b_fx[i] = shr(CURR_CW_DTFS_FX->b_fx[i], temp);
            move16();
        }
        CURR_CW_DTFS_FX->Q=PREVCW_FX.Q;
        move16();
    }


    IF (sub(CURR_CW_DTFS_FX->Q,PREVCW_FX.Q)<0)
    {
        temp = sub(PREVCW_FX.Q, CURR_CW_DTFS_FX->Q);
        tmp = s_min(shr(PREVCW_FX.lag_fx, 1),PREVCW_FX.nH_fx);
        FOR (i=0; i<=tmp; i++)
        {
            PREVCW_FX.a_fx[i] = shr(PREVCW_FX.a_fx[i], temp);
            move16();
            PREVCW_FX.b_fx[i] = shr(PREVCW_FX.b_fx[i], temp);
            move16();
        }
        PREVCW_FX.Q=CURR_CW_DTFS_FX->Q;
        move16();
    }

    DTFS_copy_fx( CURRCW_DTFS_FX, *CURR_CW_DTFS_FX);
    /* Calculating the expected alignment shift */
    alignment_fx = mult_r(*ph_offset_fx, shl(PREVCW_FX.lag_fx, 7));      /* confirmed I<2 by smv12.org, Q7 */


    IF (sub(flag,1)==0)
    alignment_fx = extract_l(L_shr(L_mult(alignment_fx, I), 1)) ;   /* Q7 */

    /* Calculating the expected alignment shift */
    find_rem((Word16)N, shr(add(PREVCW_FX.lag_fx, CURRCW_DTFS_FX->lag_fx), 1), &temp);
    temp = add(shl(temp, 7), alignment_fx);                            /*  Q7 */


    IF (temp<0)
    temp = add(temp,shl(CURRCW_DTFS_FX->lag_fx,7)); /*  Q7 */

    find_rem(temp,shl(CURRCW_DTFS_FX->lag_fx, 7),&tmp_fx); /*  Q7 */
    IF (FR_flag==0)
    {
        alignment_fx = DTFS_alignment_weight_fx(&PREVCW_FX, *CURRCW_DTFS_FX,tmp_fx, curr_lpc_fx, curr_lpc_fx, S_fx,
                                                C_fx, pf_temp1, pf_temp2, pf_temp, pf_n2);
        move16();/* Q7 */
    }
    ELSE
    {
        alignment_fx = DTFS_alignment_full_fx(PREVCW_FX, *CURRCW_DTFS_FX, *ph_offset_fx, S_fx, C_fx
        ,FR_flag
                                             );

        /*alignment_fx is in Q1, we make it Q7*/
        alignment_fx = shl(alignment_fx,6);
    }


    IF (sub(alignment_fx,shl(CURRCW_DTFS_FX->lag_fx, 7))>=0)
    {
        temp=sub(alignment_fx, shl(CURRCW_DTFS_FX->lag_fx, 7));
        tmp = shl(CURRCW_DTFS_FX->lag_fx, 7);
        exp = norm_s(tmp);
        tmp = div_s(shl(1,sub(14,exp)),tmp);/* 22-exp */
        L_tmp =L_shl(L_mult(temp,tmp),exp+1);
        tmp_fx = round_fx(L_tmp);
    }
    ELSE IF (alignment_fx<0)
    {
        temp=negate(alignment_fx);
        tmp = shl(CURRCW_DTFS_FX->lag_fx, 7);
        exp = norm_s(tmp);
        tmp = div_s(shl(1,sub(14,exp)),tmp);/* 22-exp */
        L_tmp =L_shl(L_mult(temp,tmp),exp+1);
        tmp_fx = negate( round_fx(L_tmp));
    }
    ELSE
    {
        temp=alignment_fx;
        move16();
        tmp = shl(CURRCW_DTFS_FX->lag_fx, 7);
        exp = norm_s(tmp);
        tmp = div_s(shl(1,sub(14,exp)),tmp);/* 22-exp */
        L_tmp =L_shl(L_mult(temp,tmp),exp+1);
        tmp_fx = round_fx(L_tmp);
    }

    DTFS_phaseShift_fx(CURRCW_DTFS_FX, alignment_fx, CURRCW_DTFS_FX->lag_fx, S_fx, C_fx) ;   /*  Qmin */
    DTFS_phaseShift_fx(CURR_CW_DTFS_FX, alignment_fx, CURR_CW_DTFS_FX->lag_fx, S_fx, C_fx);/*  Qmin */

    /*  Compute the cubic phase track and transform to 1-D signal */
    cubicPhase_fx(*ph_offset_fx, tmp_fx, PREVCW_FX.lag_fx , CURRCW_DTFS_FX->lag_fx, N, phase_fx) ;

    temp = shr(add(PREVCW_FX.lag_fx,CURRCW_DTFS_FX->lag_fx),1);                /*  Q0 */

    IF (FR_flag==0)
    {
        DTFS_transform_fx(PREVCW_FX,*CURRCW_DTFS_FX, phase_fx, out_fx, N, 0) ;
    }
    ELSE
    {
        DTFS_transform_fx (PREVCW_FX, *CURRCW_DTFS_FX, phase_fx, out_fx, N, 1) ;
    }


    /*  Adjust the phase offset and wrap it between 0 and 2pi */




    IF (sub(flag,2)==0)
    {
        L_temp = L_shr(L_mult(tmp_fx, I), 1);   /*  Q15 */
    }
    ELSE
    {
        L_temp = L_deposit_l(tmp_fx);                /*  Q15 */
    }


    FOR ( ; L_temp < 0; L_temp += 0x8000L)
    {
        /* empty loop */
    }
    L_temp = L_temp&0x7fff;
    move16();                         /*  fraction part */
    *ph_offset_fx = extract_l(L_temp);

    /* free(phase_fx) ; */
    free(CURRCW_DTFS_FX);
}



/*===================================================================*/
/* FUNCTION      :  ppp_extract_pitch_period_fx ()                   */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Update background noise estimate, signal energy  */
/*                  estimate, and band snrs                          */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16 []) in :  residual input, Q0                          */
/*   _ (Word16) l :  pitch lag, Q0                                   */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) out :  pitch period prototype, Q0                 */
/*   _ (Word16*) out_of_bound :  pitch lag, Q0                       */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                 */
/*   _ (Word16) spike_near_edge :   Q0                                */
/*===================================================================*/
Word16 ppp_extract_pitch_period_fx(
    const Word16 *in,           /* i : input residual     */
    Word16 *out,           /* o : output residual    */
    Word16   l,            /* i : lag                */
    Word16 *out_of_bound,   /* o : out of bound flag  */
    Word16 Qres

)
{
    Word16 i,j,k;
    Word16 spike=0,range;
    Word16 max=0;
    const Word16 *ptr=in+L_FRAME-l;
    Word32 en1 = 0,Lacc,L_tmp;
    Word16 spike_near_edge=0,scale;
    Word16 pos_max,neg_max;
    Word16 spike_pos=0,spike_neg=0;
    Word16 x,tmp,expa,fraca,expb,fracb,scale1,exp;

    pos_max = -0x8000L;
    move16();
    neg_max = 0;
    move16();
    *out_of_bound=0;
    move16();

    en1 = L_deposit_l(0);
    FOR (i=0 ; i<l; i++)
    {
        x=abs_s(ptr[i]);

        if (sub(x,max)>0)
        {
            max=x;
            move16();
            spike=i;
            move16();
        }
        en1 =L_mac0(en1, ptr[i], ptr[i]); /*  Q0 */
    }

    IF (ptr[spike]>0)
    {
        spike_pos=spike;
        move16();
        /* search for neg spike around the pos spike */
        FOR(j=spike-10; j<spike+10; j++)
        {
            k=(j+l)%l;

            if (sub(ptr[k],neg_max)<0)
            {
                neg_max=ptr[k];
                move16();
                spike_neg=k;
                move16();
            }
        }
    }
    ELSE IF (ptr[spike]<0)
    {
        spike_neg=spike;
        move16();
        /* search for pos spike around the neg spike */
        FOR(j=spike-10; j<spike+10; j++)
        {
            k=(j+l)%l;

            if (sub(ptr[k],pos_max)>0)
            {
                pos_max=ptr[k];
                move16();
                spike_pos=k;
                move16();
            }
        }
    }

    test();
    IF ((sub((l-1-s_max(spike_pos,spike_neg)),2)<=0) ||(sub(s_min(spike_pos,spike_neg),2)<=0))
    {
        *out_of_bound=1;
        move16();
        return spike_near_edge;
    }


    tmp = (Word16)(L_max(L_mult(CUTFREE_REL_RANGE_Q2,l),CUTFREE_ABS_RANGE_Q3));/* Q3 */

    IF(tmp>0)
    tmp = add(tmp,4);/* Q3 */
    ELSE
    tmp = sub(tmp,4);/* Q3 */
    range = shr(tmp,3);/* Q0 */

    test();
    IF((sub(spike,range)<0) ||  (sub(add(spike,range),l)>=0))
    {
        /* need to grab from one lag before
        ensure that there is no array bound read */

        IF(sub(sub(L_FRAME,l),l) < 0)
        {
            *out_of_bound=1;
            move16();
            return spike_near_edge;
        }
        spike_near_edge=1;
        move16();
    }

    IF(sub(spike,range)<0)
    {
        tmp = add(l,sub(spike,range));
        FOR(i=0; i<tmp; i++)
        {
            out[i]=ptr[i];
            move16();
        }
        /* Grab Remaining From One Lag Before */
        ptr-=l;
        FOR(; i<l; i++)
        {
            out[i]=ptr[i];
            move16();
        }
    }
    ELSE IF (sub(add(spike,range),l) >= 0)
    {
        tmp = sub(spike,range);
        FOR(i=0; i<tmp; i++)
        {
            out[i]=ptr[i];
            move16();
        }
        /* Grab Remaining From One Lag Before */
        IF (ptr-l+i>=in)
        {
            FOR(ptr-=l; i<l; i++)
            {
                out[i]=ptr[i];
                move16();
            }
        }
        ELSE
        {
            FOR(; i<l; i++)
            {
                out[i]=ptr[i];
                move16();
            }
        }
    }
    ELSE
    {
        FOR(i=0; i<l; i++)
        {
            out[i]=ptr[i];
            move16();
        }
    }
    /* Energy adjustment added to eliminate artifacts at the end of voicing */
    Lacc = L_deposit_l(0);
    FOR (i=0; i<l; i++)
    {
        Lacc = L_mac0(Lacc,out[i],out[i]); /*  Q0 */
    }


    IF (L_sub(en1,Lacc)<0)
    {
        /*  Ltmp=sqrt_divide_dp(en1, Lacc, 0, &n,1); */
        /*  scale=round_fx(L_shl(Ltmp,sub(31,n))); in Q15 */
        expa = norm_l(en1);
        fraca = extract_h(L_shl(en1,expa));
        expa = sub(30, add(expa,Qres));


        expb = norm_l(Lacc);
        fracb = round_fx(L_shl(Lacc,expb));
        expb = sub(30, add(expb,Qres));

        scale1 = shr(sub(fraca,fracb),15);
        fracb = shl(fracb,scale1);
        expb = sub(expb,scale1);

        tmp = div_s(fracb,fraca);
        exp = sub(expb,expa);

        L_tmp = Isqrt_lc(L_deposit_h(tmp),&exp); /* Q(31-exp) */
        scale = round_fx(L_tmp);/* Q15 */

        FOR (i=0; i<l; i++)
        {
            out[i] = mult_r(out[i],scale);
            move16();
        }
    }
    return spike_near_edge;
}
/*===========================================================================*/
/* FUNCTION      :  DTFS_peaktoaverage_fx ()                                 */
/*---------------------------------------------------------------------------*/
/* PURPOSE       :                                                           */
/*---------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                        */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx: a_fx/b_fx in Q(X_fx.Q), lag in Q0    */
/*---------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                        */
/*   _ (Word32 *) pos_fx :  in *Qpos                                         */
/*   _ (Word32 *) neg_fx :  in *Qneg                                         */
/*   _ (Word16 *) Qpos: Q valule of output *pos_fx                           */
/*   _ (Word16 *) Qneg: Q valule of output *neg_fx                           */
/*---------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                  */
/*                    _ None                                                 */
/*---------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                                */
/*===========================================================================*/
void DTFS_peaktoaverage_fx( DTFS_STRUCTURE_FX X_fx,Word32 *pos_fx, Word16 *Qpos, Word32 *neg_fx, Word16 *Qneg)
{
    Word32 L_sum;
    Word32 maxPosEn_fx=0, maxNegEn_fx=0, L_temp;
    Word16 i;
    Word16 time_fx[256];
    Word16 expa,expb,fraca,fracb,scale;
    Word16 exp,tmp;
    Word32 L_tmp;

    L_sum=DTFS_getEngy_P2A_fx(&X_fx); /*  2Q */
    DTFS_fast_fs_inv_fx(&X_fx,time_fx , 256, 8);

    FOR (i=0; i<256; i++)
    {
        L_temp = L_mult(time_fx[i], time_fx[i]);         /*  Q(1) */

        IF (time_fx[i] >= 0)
        {
            if (L_sub(L_temp,maxPosEn_fx)>0)
            {
                maxPosEn_fx = L_temp ; /*  Q(1) */
            }
        }
        ELSE
        {
            if (L_sub(L_temp,maxNegEn_fx)>0)
            {
                maxNegEn_fx = L_temp ; /*  Q(1) */
            }
        }
    }


    IF (L_sum==0)
    {
        *pos_fx=*neg_fx=0;
        move16();
        move16();
    }
    ELSE
    {

        expa = norm_l(maxPosEn_fx);
        fraca = extract_h(L_shl(maxPosEn_fx,expa));
        expa = sub(30,add(expa,1));


        expb = norm_l(L_sum);
        fracb = round_fx(L_shl(L_sum,expb));
        expb = sub(30,add(expb,shl(X_fx.Q,1)));


        scale = shr(sub(fraca,fracb),15);
        fracb = shl(fracb,scale);
        expb = sub(expb,scale);

        tmp = div_s(fracb,fraca);
        exp = sub(expb,expa);

        L_tmp = Isqrt_lc(L_deposit_h(tmp),&exp); /* Q(31-exp) */
        *pos_fx = L_tmp;
        *Qpos = sub(31, exp);


        expa = norm_l(maxNegEn_fx);
        fraca = extract_h(L_shl(maxNegEn_fx,expa));
        expa = 30-expa- 1;


        expb = norm_l(L_sum);
        fracb = round_fx(L_shl(L_sum,expb));
        expb = 30-expb - (2*X_fx.Q);


        scale = shr(sub(fraca,fracb),15);
        fracb = shl(fracb,scale);
        expb = sub(expb,scale);

        tmp = div_s(fracb,fraca);
        exp = sub(expb,expa);

        L_tmp = Isqrt_lc(L_deposit_h(tmp),&exp); /* Q(31-exp) */
        *neg_fx = L_tmp;
        *Qneg = 31 - exp;

    }
}

/*===================================================================*/
/* FUNCTION      :  struct DTFS_fx:: rshiftHarmBand_fx()             */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Right-shift harmonics in band to align Q factor  */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) lband: lower band boundary, Q15                      */
/*   _ (Word16) hband: upper band boundary, Q15                      */
/*   _ (Word16) shift: right shift value, Q0 (must be <0)            */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (struct DTFS_fx) X :  prototype in polar domain               */
/*                (Word16) lag: length of prototype in time domain   */
/*                (Word16 []) a: amplitude of harmonics, normalized  */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
/* NOTE: This function should be called after two consecutive calls  */
/*       to setEngyHarm_fx, because the norm factor outputs from     */
/*       the two previous calls might be different                   */
/*===================================================================*/
void rshiftHarmBand_fx( DTFS_STRUCTURE_FX *X_fx,Word16 lband_fx, Word16 hband_fx, Word16 shift_fx)
{
    Word16 k_fx, HalfLag_fx;
    Word16 low_fx, high_fx;

    low_fx=mult(lband_fx,X_fx->lag_fx);  /*  low=lband*lag, Q0 */
    high_fx=mult(hband_fx,X_fx->lag_fx); /*  high=hband*lag, Q0 */
    HalfLag_fx = s_min(high_fx,shl(X_fx->nH_fx,1));

    FOR (k_fx=low_fx+1; k_fx<=HalfLag_fx; k_fx++)
    {
        X_fx->a_fx[k_fx]=(Word16)shift_r(X_fx->a_fx[k_fx],shift_fx);
        move16(); /*  right shift and round */
    }
}

/*===================================================================*/
/* FUNCTION      :  GetSinCosTab_fx ()                               */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Compute sine & cosine table given pitch lag,     */
/*                  by interpolating the 512-entry cosine table.     */
/*                  sin(2pi/4L*n) & cos(2pi/4L*n) for n=0,1,... 4L-1 */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) L :  Pitch lag, Q0                                   */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) sinTab, Q15 : sin(2pi/4L*n), n=0,1,...,4L-1       */
/*   _ (Word16 []) cosTab, Q15 : cos(2pi/4L*n), n=0,1,...,4L-1       */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
/* NOTE: This function interpolates cos_table for better accuracy    */
/*===================================================================*/
void GetSinCosTab_fx(Word16 L, Word16 *sinTab, Word16 *cosTab)
{
    Word16 i, L4;
    Word16 dl, t1, w, dt;
    /* Word16 t2; */
    Word32 invL; /*  1/4L in Q25 */
    Word32 Ltemp, Lw;
    Word32 L_tmp;

    invL = L_shr(inverse_table[L],4);

    L4=shl(L,2);
    Lw = L_deposit_l(0);
    FOR (i=0 ; i<L4; i++)
    {
        w = extract_h(Lw); /*  w in Q9 */
        dl = extract_l(Lw); /*  dl has 16 bits left-over */
        w = s_and(w,511);
        move16();
        move16();
        /*  t1=cos_table[w%512]; */
        t1=cos_table[w];
        /*   t2=cos_table[add(w,1)%512]; */
        dt = cos_diff_table[w];
        /*t2=cos_table[s_and(add(w,1),511)]; */
        /*dt = sub(t2,t1); */ /*  Q15 */

        IF(dl < 0)
        {
            L_tmp = L_add(65536,dl);
            Ltemp = (L_tmp*dt);
        }
        ELSE
        {
            Ltemp = (Word32)L_mult0(dt,dl); /*  Ltemp in Q31 */
        }
        cosTab[i] = add(t1,(Word16)L_shr(Ltemp,16));
        move16();/*  interpolated cos(i*2pi/4L) */

        Ltemp = L_add(Lw,0x1800000); /*  add 0.75 in Q25, which is 3pi/2 to convert sin to cos */
        w = extract_h(Ltemp); /*  w is equivalent cos index */
        /*dl = extract_l(Ltemp); */ /*  dl is 16 bit left-over for interpolation */
        w = s_and(w,511);


        /*  t1=cos_table[w%512]; */
        t1=cos_table[w];
        move16();
        /*   t2=cos_table[add(w,1)%512]; */
        dt = cos_diff_table[w];
        /*t2=cos_table[s_and(add(w,1),511)];move16(); */
        /*dt = sub(t2,t1); */ /*  dt=t2-t1, Q15 */

        IF(dl<0)
        {
            L_tmp = L_add(65536,dl);
            Ltemp = (L_tmp*dt);

        }
        ELSE
        {
            Ltemp = L_mult0(dt,dl); /*  Ltemp in Q31 */
        }
        sinTab[i]=add(t1,(Word16)L_shr(Ltemp,16));
        move16();/*  t1 is interpolated cos(w) */

        Lw=L_add(Lw,invL); /*  Lw=(i+1)/4L, Q25 */
    }
}



/*
 * The decimation-in-time complex FFT/IFFT is implemented below.
 * The input complex numbers are presented as real part followed by
 * imaginary part for each sample.  The counters are therefore
 * incremented by two to access the complex valued samples.
 */
static void c_fft_wi_fx(Word16 * farray_ptr_fx, Word16 size, Word16 stage, Word16 isign)
{

    Word16 i, j, k, ii, jj, kk, ji, kj;
    Word32 ftmp_fx, ftmp_real_fx, ftmp_imag_fx;
    Word16 tmp, tmp1, tmp2,temp_sand;
    Word16 n_2, K;
    Word16 ii_table[8];


    move16();
    move16();

    K=0;

    FOR( k = 256; k > 0; k -= size)
    {
        K=add(K,1); /*  K=512/size */
    }
    n_2=shr(size,1);
    FOR (i=1; i<=stage; i++) ii_table[i-1]=shr(size,i);

    /* Rearrange the input array in bit reversed order */
    j = 0;
    move16();
    FOR (i = 0 ; i < size - 2; i = i + 2)
    {
        move16();
        move16();
        move16();
        move16();
        move16();
        move16();

        IF (sub(j,i) > 0)
        {
            ftmp_fx = *(farray_ptr_fx + i);
            *(farray_ptr_fx + i) = *(farray_ptr_fx + j);
            *(farray_ptr_fx + j) = (Word16) ftmp_fx;

            ftmp_fx = *(farray_ptr_fx + i + 1);
            *(farray_ptr_fx + i + 1) = *(farray_ptr_fx + j + 1);
            *(farray_ptr_fx + j + 1) = (Word16)ftmp_fx;
        }

        k = n_2;
        move16();
        WHILE (j >= k)
        {
            j = sub(j, k);
            k = shr(k, 1);
        }
        j += k;
    }

    /* The FFT part */



    IF (isign == 1)
    {
        FOR (i = 0; i < stage; i++)                         /* i is stage counter */
        {
            jj = shl(2, i);     /* FFT size */
            kk = shl(jj, 1);    /* 2 * FFT size */

            move16();

            ii = ii_table[i];   /* 2 * number of FFT's */

            FOR (j = 0; j < jj; j = j + 2)                     /* j is sample counter */
            {
                ji = j * ii;    /* ji is phase table index */

                FOR (k = j; k < size; k = k + kk)                 /* k is butterfly top */
                {
                    kj = add(k, jj);    /* kj is butterfly bottom */
                    temp_sand = s_and((ji*K+384),511);
                    /* Butterfly computations */
                    /*  ftmp_real_fx = L_sub(L_mult(*(farray_ptr_fx + kj), cos_table[ji*K]), */
                    /*                   L_mult(*(farray_ptr_fx + kj + 1), cos_table[(ji*K+384)%512])); */
                    ftmp_real_fx = L_msu(L_mult(*(farray_ptr_fx + kj), cos_table[ji*K]),
                                         *(farray_ptr_fx + kj + 1), cos_table[temp_sand]);

                    /*   ftmp_imag_fx = L_add(L_mult(*(farray_ptr_fx + kj + 1), cos_table[ji*K]), */
                    /*                    L_mult(*(farray_ptr_fx + kj), cos_table[(ji*K+384)%512])); */
                    ftmp_imag_fx = L_mac(L_mult(*(farray_ptr_fx + kj + 1), cos_table[ji*K]),
                                         *(farray_ptr_fx + kj), cos_table[temp_sand]);

                    tmp1 = round_fx(ftmp_real_fx);
                    tmp2 = round_fx(ftmp_imag_fx);

                    tmp = sub(*(farray_ptr_fx + k), tmp1);
                    *(farray_ptr_fx + kj) = shr(tmp, 1);
                    move16();

                    tmp = sub(*(farray_ptr_fx + k + 1), tmp2);
                    *(farray_ptr_fx + kj + 1) = shr(tmp, 1);
                    move16();

                    tmp = add(*(farray_ptr_fx + k), tmp1);
                    *(farray_ptr_fx + k) = shr(tmp, 1);
                    move16();

                    tmp = add(*(farray_ptr_fx + k + 1), tmp2);
                    *(farray_ptr_fx + k + 1) = shr(tmp, 1);
                    move16();
                }
            }
        }

        /* The IFFT part */
    }
    ELSE
    {
        FOR (i = 0; i < stage; i++)                         /* i is stage counter */
        {
            jj = shl(2, i);     /* FFT size */
            kk = shl(jj, 1);    /* 2 * FFT size */
            ii = ii_table[i];   /* 2 * number of FFT's */

            FOR (j = 0; j < jj; j = j + 2)                     /* j is sample counter */
            {
                ji = j * ii;    /* ji is phase table index */

                FOR (k = j; k < size; k = k + kk)                 /* k is butterfly top */
                {
                    kj = add(k, jj);    /* kj is butterfly bottom */
                    temp_sand = s_and((ji*K+384),511);
                    /* Butterfly computations */
                    /*  ftmp_real_fx = L_add(L_mult(*(farray_ptr_fx + kj), cos_table[ji*K]), */
                    /*                   L_mult(*(farray_ptr_fx + kj + 1), cos_table[(ji*K+384)%512])); */
                    ftmp_real_fx = L_mac(L_mult(*(farray_ptr_fx + kj), cos_table[ji*K]),
                    *(farray_ptr_fx + kj + 1), cos_table[temp_sand]);

                    /*    ftmp_imag_fx = L_sub(L_mult(*(farray_ptr_fx + kj + 1), cos_table[ji*K]), */
                    /*                   L_mult(*(farray_ptr_fx + kj), cos_table[(ji*K+384)%512])); */
                    ftmp_imag_fx = L_msu(L_mult(*(farray_ptr_fx + kj + 1), cos_table[ji*K]),
                    *(farray_ptr_fx + kj), cos_table[temp_sand]);

                    tmp1 = round_fx(ftmp_real_fx);
                    tmp2 = round_fx(ftmp_imag_fx);

                    *(farray_ptr_fx + kj) = sub(*(farray_ptr_fx + k), tmp1);
                    move16();
                    *(farray_ptr_fx + kj + 1) = sub(*(farray_ptr_fx + k + 1), tmp2);
                    move16();
                    *(farray_ptr_fx + k) = add(*(farray_ptr_fx + k), tmp1);
                    move16();
                    *(farray_ptr_fx + k + 1) = add(*(farray_ptr_fx + k + 1), tmp2);
                    move16();
                }
            }
        }
    }

}                               /* end of c_fft () */


void r_fft_4_fx(Word16 * farray_ptr_fx, Word16 size, Word16 stage, Word16 isign)
{

    Word16 ftmp1_real_fx, ftmp1_imag_fx, ftmp2_real_fx, ftmp2_imag_fx;
    Word32 Lftmp1_real_fx, Lftmp1_imag_fx;
    Word16 i, j,temp_sand;
    Word32 Ltmp1_fx, Ltmp2_fx;
    Word16 n_2, k, K;

    n_2 = shr(size,1);
    K=0;
    move16();

    FOR (k = 256; k > 0; k -= size)
    {
        K = add(K,1); /*  K=512/size */
    }

    /* The FFT part */
    IF (isign == 1)
    {
        /* Perform the complex FFT */
        c_fft_wi_fx(farray_ptr_fx, size, stage, isign);

        /* First, handle the DC and foldover frequencies */
        ftmp1_real_fx = *farray_ptr_fx;
        ftmp2_real_fx = *(farray_ptr_fx + 1);
        *farray_ptr_fx = add(ftmp1_real_fx, ftmp2_real_fx);
        *(farray_ptr_fx + 1) = sub(ftmp1_real_fx, ftmp2_real_fx);

        /* Now, handle the remaining positive frequencies */
        j = size - 2;
        FOR (i = 2; i <= n_2; i = i + 2 )
        {
            ftmp1_real_fx = add(*(farray_ptr_fx + i), *(farray_ptr_fx + j));
            ftmp1_imag_fx = sub(*(farray_ptr_fx + i + 1), *(farray_ptr_fx + j + 1));
            ftmp2_real_fx = add(*(farray_ptr_fx + i + 1), *(farray_ptr_fx + j + 1));
            ftmp2_imag_fx = sub(*(farray_ptr_fx + j), *(farray_ptr_fx + i));

            Lftmp1_real_fx = L_deposit_h(ftmp1_real_fx);
            Lftmp1_imag_fx = L_deposit_h(ftmp1_imag_fx);
            temp_sand = s_and((i*K+384),511);
            /*  Ltmp1_fx = L_sub(L_mult(ftmp2_real_fx, cos_table[i*K]), L_mult(ftmp2_imag_fx, cos_table[(i*K+384)%512])); */
            Ltmp1_fx = L_msu(L_mult(ftmp2_real_fx, cos_table[i*K]), ftmp2_imag_fx, cos_table[temp_sand]);
            *(farray_ptr_fx + i) = round_fx(L_shr(L_add(Lftmp1_real_fx, Ltmp1_fx), 1));

            /*   Ltmp1_fx = L_add(L_mult(ftmp2_imag_fx, cos_table[i*K]), L_mult(ftmp2_real_fx, cos_table[(i*K+384)%512])); */
            Ltmp1_fx = L_mac(L_mult(ftmp2_imag_fx, cos_table[i*K]), ftmp2_real_fx, cos_table[temp_sand]);
            *(farray_ptr_fx + i + 1) = round_fx(L_shr(L_add(Lftmp1_imag_fx, Ltmp1_fx), 1));

            /*    Ltmp1_fx = L_add(L_mult(ftmp2_real_fx, cos_table[j*K]), L_mult(ftmp2_imag_fx, cos_table[(j*K+384)%512])); */
            Ltmp1_fx = L_mac(L_mult(ftmp2_real_fx, cos_table[j*K]), ftmp2_imag_fx, cos_table[temp_sand]);
            *(farray_ptr_fx + j) = round_fx(L_shr(L_add(Lftmp1_real_fx, Ltmp1_fx), 1));

            /*    Ltmp1_fx = L_add(L_negate(L_mult(ftmp2_imag_fx, cos_table[j*K])), L_mult(ftmp2_real_fx, cos_table[(j*K+384)%512])); */
            Ltmp1_fx = L_msu(L_mult(ftmp2_real_fx, cos_table[temp_sand]), ftmp2_imag_fx, cos_table[j*K]);
            Ltmp2_fx = L_sub(Ltmp1_fx, Lftmp1_imag_fx);
            *(farray_ptr_fx + j + 1) = round_fx(L_shr(Ltmp2_fx, 1));
            j = size - i;
        }

    }
    ELSE
    {

        /* First, handle the DC and foldover frequencies */

        move16();
        move16();

        ftmp1_real_fx = *farray_ptr_fx;
        ftmp2_real_fx = *(farray_ptr_fx + 1);
        *farray_ptr_fx = shr(add(ftmp1_real_fx, ftmp2_real_fx), 1);
        move16();
        *(farray_ptr_fx + 1) = shr(sub(ftmp1_real_fx, ftmp2_real_fx), 1);
        move16();

        /* Now, handle the remaining positive frequencies */
        FOR (i = 2; i <= n_2; i += 2)
        {
            j = sub(size, i);

            ftmp1_real_fx = add(*(farray_ptr_fx + i), *(farray_ptr_fx + j));
            ftmp1_imag_fx = sub(*(farray_ptr_fx + i + 1), *(farray_ptr_fx + j + 1));
            ftmp2_real_fx = negate(add(*(farray_ptr_fx + j + 1), *(farray_ptr_fx + i + 1)));
            ftmp2_imag_fx = negate(sub(*(farray_ptr_fx + j), *(farray_ptr_fx + i)));

            Lftmp1_real_fx = L_deposit_h(ftmp1_real_fx);
            Lftmp1_imag_fx = L_deposit_h(ftmp1_imag_fx);
            temp_sand = s_and((i*K+384),511);
            /*  Ltmp1_fx = L_add(L_mult(ftmp2_real_fx, cos_table[i*K]), L_mult(ftmp2_imag_fx, cos_table[(i*K+384)%512])); */
            Ltmp1_fx = L_mac(L_mult(ftmp2_real_fx, cos_table[i*K]), ftmp2_imag_fx, cos_table[    temp_sand]);
            *(farray_ptr_fx + i) = round_fx(L_shr(L_add(Lftmp1_real_fx, Ltmp1_fx), 1));

            /*  Ltmp1_fx = L_sub(L_mult(ftmp2_imag_fx, cos_table[i*K]), L_mult(ftmp2_real_fx, cos_table[(i*K+384)%512])); */
            Ltmp1_fx = L_msu(L_mult(ftmp2_imag_fx, cos_table[i*K]), ftmp2_real_fx, cos_table[    temp_sand]);
            *(farray_ptr_fx + i + 1) = round_fx(L_shr(L_add(Lftmp1_imag_fx, Ltmp1_fx), 1));

            /*  Ltmp1_fx = L_sub(L_mult(ftmp2_real_fx, cos_table[j*K]), L_mult(ftmp2_imag_fx, cos_table[(j*K+384)%512])); */
            Ltmp1_fx = L_msu(L_mult(ftmp2_real_fx, cos_table[j*K]), ftmp2_imag_fx, cos_table[temp_sand]);
            *(farray_ptr_fx + j) = round_fx(L_shr(L_add(Lftmp1_real_fx, Ltmp1_fx), 1));

            /*  Ltmp1_fx = L_negate(L_add(L_mult(ftmp2_imag_fx, cos_table[j*K]), L_mult(ftmp2_real_fx, cos_table[(j*K+384)%512]))); */
            Ltmp1_fx = L_negate(L_mac(L_mult(ftmp2_imag_fx, cos_table[j*K]), ftmp2_real_fx, cos_table[temp_sand]));
            Ltmp2_fx = L_sub(Ltmp1_fx, Lftmp1_imag_fx);
            *(farray_ptr_fx + j + 1) = round_fx(L_shr(Ltmp2_fx, 1));
        }

        /* Perform the complex IFFT */
        c_fft_wi_fx(farray_ptr_fx, size, stage, isign);

    }

}


/*===================================================================*/
/* FUNCTION      :  struct DTFS_fx::copy_phase_fx ()                 */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Retain the amplitudes of a prototype X2, but copy*/
/*                  the phases of another prototype X1 of same length*/
/*                  over to make a new prototype                     */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_fx) X1 :  prototype in Cartesian domain          */
/*                (Word16) X1.lag: length of prototype in time domain*/
/*                (Word16 []) X1.a,b: re/im of harmonics, normalized */
/*                (Word16) X1.Q: norm factor of X2.a/b               */
/*   _ (struct DTFS_fx) X2 :  prototype in polar domain              */
/*                (Word16) X2.lag: should be same as X1.lag          */
/*                (Word16 []) X2.a:amplitude of harmonics, normalized*/
/*                (Word16 []) X2.b: phase of harmonics, don't care   */
/*                (Word16) X2.Q: norm factor of X2.a                 */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*   _ (struct DTFS_fx) X :  prototype in Cartesian domain           */
/*                    The amplitudes of this prototype are from X2   */
/*                    and the phases are from X1                     */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/
/* X.a[k]=2*X2.a[k]/sqrt(X1.a[k]^2+X1.b[k]^2)*X1.a[k]                */
/* X.b[k]=2*X2.a[k]/sqrt(X1.a[k]^2+X1.b[k]^2)*X1.b[k]                */
/*===================================================================*/
void copy_phase_fx( DTFS_STRUCTURE_FX *X1_fx, DTFS_STRUCTURE_FX X2_fx, DTFS_STRUCTURE_FX *retX_fx)
{
    /* DTFS_fx X; */
    Word16 k, q, sn, cn, HalfLag;
    Word16 d1h, d1l;
    Word32 Ltemp_fx,L_tmp;
    Word32 Lacc_fx;
    Word16 exp,tmp,exp1;

    move16();

    retX_fx->lag_fx=X1_fx->lag_fx;
    retX_fx->Q = sub(X2_fx.Q,1); /*  equivalent to 2x MIN_FX(shr(sub(X_fx->lag_fx,1),1),X_fx->nH_fx) */
    HalfLag = s_min(shr(X1_fx->lag_fx,1),X1_fx->nH_fx);
    move16();
    FOR ( k=1; k<=HalfLag; k++ )
    {
        Lacc_fx=L_mult(X1_fx->a_fx[k],X1_fx->a_fx[k]);
        Lacc_fx = L_mac(Lacc_fx,X1_fx->b_fx[k],X1_fx->b_fx[k]); /*  2*Q+1 */

        exp = norm_l(Lacc_fx);
        tmp = extract_h(L_shl(Lacc_fx,exp));
        exp1 = sub(sub(30,exp),2*X1_fx->Q+1);

        IF(tmp)
        tmp = div_s(16384,tmp); /* 15+exp1 */
        ELSE
        tmp = 0;
        tmp = shr(tmp,1);
        q = 15+exp1+16-1;

        IF(tmp)
        {
            exp = norm_s(tmp);
            tmp = shl(tmp, exp);
            tmp = div_s(16384, tmp);
            L_tmp = L_deposit_h(tmp);
            Ltemp_fx = Isqrt_lc(L_tmp,&exp); /* Q(31-exp) */
        }
        ELSE
        Ltemp_fx = 0;

        if (s_and(q, 1))
            Ltemp_fx = Mult_32_16(Ltemp_fx, 23170); /*  23170 is 1/sqrt(2) in Q15 */

        q = shr(q,1); /*  Ltemp_fx in Q(q+16) */

        d1h=extract_h(Ltemp_fx);
        d1l=extract_l(Ltemp_fx);
        Ltemp_fx = L_mult0(X1_fx->b_fx[k],d1l);
        Ltemp_fx = L_add(L_shr(Ltemp_fx,15),L_mult(X1_fx->b_fx[k],d1h)); /*  sin(w) in Q(q+16+Q-15) */
        sn = round_fx(L_shl(Ltemp_fx,sub(30,add(q,X1_fx->Q)))); /*  Q15 */
        retX_fx->b_fx[k] = mult_r(X2_fx.a_fx[k],sn); /*  X2_fx.Q */

        Ltemp_fx = L_mult0(X1_fx->a_fx[k],d1l);
        Ltemp_fx = L_add(L_shr(Ltemp_fx,15),L_mult(X1_fx->a_fx[k],d1h)); /*  cos(w) in Q(q+Q+1) */
        cn = round_fx(L_shl(Ltemp_fx,sub(30,add(q,X1_fx->Q)))); /*  Q15 */
        retX_fx->a_fx[k] = mult_r(X2_fx.a_fx[k],cn); /*  X2_fx.Q         */

    }
    k=sub(k,1);


    IF (s_and(X1_fx->lag_fx,1)==0)
    {
        retX_fx->a_fx[k] = shr(retX_fx->a_fx[k],1);
        move16();
        retX_fx->b_fx[k] = shr(retX_fx->b_fx[k],1);
        move16();
    }
}



/*===================================================================*/
/* FUNCTION      :  getSpEngyFromResAmp_fx ()                        */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Get band energy                                  */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx :  prototype in polar domain  */
/*                (Word16) lag: length of prototype in time domain   */
/*                (Word16 []) a: amplitude of harmonics, normalized  */
/*                (Word16) Q: norm factor of a                       */
/*   _ (Word16 []) curr_lpc: LPC coefficients, Q12                   */
/*   _ (Word16) lband: lower frequency bound, Q15                    */
/*   _ (Word16) hband: upper frequency bound, Q15                    */
/*   _ (Word16 []) sin_tab: sine table based on lag, Q15             */
/*   _ (Word16 []) cos_tab: cosine table based on lag, Q15           */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*   _ (Word32) en: energy of the specified frequency band,          */
/*                  Q factor is 2Q-13                                */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX                                                  */
/*===================================================================*/
Word32 getSpEngyFromResAmp_fx( DTFS_STRUCTURE_FX *X_fx,Word16 lband, Word16 hband,
                               const Word16 *curr_lpc, Word16 *sin_tab,
                               Word16 *cos_tab)
{
    Word16 i, k, k4, n, M_fx, HalfLag;
    Word16 fdiff, freq;
    Word32 Ltemp;
    Word32 Lacc;
    Word32 Re, Im;  /*  Q27 */
    Word32 en;
    Word16 exp,tmp,expa,fraca,expb,fracb,scale;
    Word32 L_tmp;

    en = L_deposit_l(0);

    if (sub(hband,X_fx->upper_cut_off_freq_fx)==0)
    {
        hband = 0x2803;
        move16(); /*  4001.0/12800 in Q15 */
    }
    M_fx=shl(X_fx->lag_fx,2); /*  M_fx=4*lag */

    /*  Ltemp=invert_dp(X_fx->lag_fx, 4, &n,1); : Ltemp=1/lag, Q(61-n) */
    /* fdiff=round_fx(L_shl(Ltemp,sub(n,26))); : fdiff=1/lag, Q19 */

    exp = norm_s(X_fx->lag_fx);
    tmp = div_s(shl(1,sub(14,exp)),X_fx->lag_fx);
    L_tmp = L_shl(tmp ,add(exp,6));
    fdiff = round_fx(L_tmp);

    HalfLag = s_min(shr(X_fx->lag_fx,1),X_fx->nH_4kHz_fx);
    FOR (k=0; k<=HalfLag; k++)
    {
        Ltemp=L_mult(fdiff,k); /*  Ltemp=k*fdiff, Q20 */
        freq=extract_h(L_shl(Ltemp,11)); /*  k*fdiff in Q15 */

        test();
        IF (sub(freq,hband)<0 && sub(freq,lband)>=0)
        {
            Lacc = L_add(0x10000000, 0);     /*  Re=1.0, Q28 */
            k4=shl(k,2); /*  k4=4*k */

            n=k4;
            move16();
            FOR (i=0; i<M+1; i++)
            {
                /*  Compute Re */
                Lacc=L_mac(Lacc,curr_lpc[i],cos_tab[n%M_fx]); /*  Q28 */
                n=add(n,k4); /*  n=4*i*k */
            }
            Re = L_shr(Lacc,1); /*  Q27 */

            n=k4;
            move16();
            Lacc = L_deposit_l(0);
            FOR (i=0; i<M+1; i++)
            {
                /*  Compute Im */
                Lacc=L_msu(Lacc,curr_lpc[i],sin_tab[n%M_fx]); /*  Q28 */
                n=add(n,k4); /*  n=4*i*k */
            }
            Im=L_shr(Lacc,1); /*  Q27 */
            /* Lacc=L_add(L_mult_ll(Re,Re),(Word32)L_mult_ll(Im,Im)); : Lacc=Re^2+Im^2 in Q23 */
            Lacc=L_add(Mult_32_32(Re,Re),Mult_32_32(Im,Im)); /*  Lacc=Re^2+Im^2 in Q23 */
            Ltemp=L_mult0(X_fx->a_fx[k],X_fx->a_fx[k]); /*  2*a[k]^2 in 2Q */
            /* Ltemp=(Word32)L_sat32_40(divide_dp(Ltemp,Lacc,-19,1)); : Ltemp in Q(2Q-13) */

            if(Lacc<0)
            {
                Lacc = L_negate(Lacc);
            }

            IF(Lacc)
            {

                expa = norm_l(Lacc);
                fraca = extract_h(L_shl(Lacc,expa));
                expa = sub(30,add(expa,23));


                expb = norm_l(Ltemp);
                fracb = round_fx(L_shl(Ltemp,expb));
                expb =  sub(30,add(expb, shl(X_fx->Q,1)));

                scale = shr(sub(fraca,fracb),15);
                fracb = shl(fracb,scale);
                expb = sub(expb,scale);

                tmp = div_s(fracb,fraca); /* 15-exp */
                exp = sub(expb,expa);
                Ltemp = L_shl(tmp, sub(add(shl(X_fx->Q,1),exp), 27)) ;

            }
            ELSE
            {
                Ltemp = L_deposit_l(0);
            }

            test();
            IF (X_fx->lag_fx%2==0 && sub(k,shr(X_fx->lag_fx,1))==0)
            en=L_add(en,L_shr(Ltemp,1));
            ELSE
            en=L_add(en,Ltemp); /*  en in 2Q-13 */
        }

    }
    return(en); /*  en in 2Q-13 */
}
/*===================================================================*/
/* FUNCTION      :  DTFS_poleFilter_fx()                             */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  pole filtering                                   */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) lpc[] :  lpc coefficients in Q12                     */
/*   _ (Word16) N     :  lpc order                                   */
/*   _ (Word16) X_fx->lag_fx:  in Q0                                 */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ None                                                          */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ (Word16) X_fx->a_fx[] :    in Q(X_fx->Q)                      */
/*   _ (Word16) X_fx->b_fx[] :    in Q(X_fx->Q)                      */
/*   _ (Word16) X_fx->Q:       in Q0                                 */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void DTFS_poleFilter_fx( DTFS_STRUCTURE_FX *X_fx,Word16 *LPC, Word16 N, Word16 *S_fx, Word16 *C_fx)
{
    Word16 temp, temp1, temp2, HalfLag ;
    Word32 sum1_fx, sum2_fx;
    Word32 sum, L_temp1, L_temp2;
    Word16 k, n, na, nb;
    Word16 Qmin, Qab[MAXLAG_WI];
    Word16 exp,tmp;

    Qmin = 32767;
    move16();
    HalfLag = s_min(shr(X_fx->lag_fx, 1),X_fx->nH_fx);
    FOR ( k=0; k<=HalfLag; k++ )
    {
        temp2 = k;
        move16();
        /* Calculate sum1 and sum2 */
        sum1_fx = L_deposit_h(4096);      /*  1: Q(12+15+1)  */

        sum2_fx = L_deposit_l(0);
        FOR ( n=0 ; n<N ; n++  )
        {
            sum1_fx = L_mac(sum1_fx, LPC[n], C_fx[(4*temp2)%(4*X_fx->lag_fx)]) ;   /* Q(12+15+1) */
            sum2_fx = L_mac(sum2_fx, LPC[n], S_fx[(4*temp2)%(4*X_fx->lag_fx)]) ;   /* Q(12+15+1) */
            temp2 = add(temp2, k);
        }

        temp1 = round_fx(sum1_fx);   /* Q(12+15+1-16)=Q(12) */
        temp2 = round_fx(sum2_fx);   /*               Q(12) */

        /* Calculate the circular convolution */
        sum = L_mac(L_mult(temp1, temp1), temp2, temp2); /*  Q(12+12+1)=Q(25) */

        L_temp1 = L_mult(temp1, X_fx->a_fx[k]);
        L_temp1 = L_mac(L_temp1, temp2, X_fx->b_fx[k]); /*  Q(12+Q+1)=Q(13+Q) */
        L_temp2 = L_mult(temp1, X_fx->b_fx[k]);
        L_temp2 = L_msu(L_temp2, temp2, X_fx->a_fx[k]); /*  Q(12+Q+1)=Q(13+Q) */

        IF(sum)
        {
            exp = norm_l(sum);
            temp1 = exp;
            tmp = extract_h(L_shl(sum,exp));
            exp = sub(sub(30,exp),25);
            tmp = div_s(16384,tmp); /* Q(15+exp) */
            sum = L_shl(tmp ,16 );
            temp = round_fx(sum);
        }
        ELSE
        {
            sum =0;
            move16();
            temp =0;
            move16();
            temp1 =0;
            move16();

        }
        sum1_fx = Mult_32_16(L_temp1, temp);      /*  Q(13+Q+20-temp1-15)=Q(Q-temp1+18) */
        sum2_fx = Mult_32_16(L_temp2, temp);      /*                      Q(Q-temp1+18) */

        /*  normalization */
        na = norm_l(sum1_fx);
        if (sum1_fx==0)
        {
            na = 31;
            move16();
        }
        nb = norm_l(sum2_fx);
        if (sum2_fx==0)
        {
            nb = 31;
            move16();
        }

        if (sub(na, nb)<0)
        {
            nb=na;
            move16();
        }
        nb=sub(nb,1); /*  leave one more sign bit */
        X_fx->a_fx[k] = round_fx(L_shl(sum1_fx, nb));   /* Q(Q-temp1+22+nb-16)=Q(Q-temp1+nb+2) */
        X_fx->b_fx[k] = round_fx(L_shl(sum2_fx, nb));   /*                    Q(Q-temp1+nb+2) */

        Qab[k] = add(sub(add(nb, 2), temp1),X_fx->Q);

        if (sub(Qab[k], Qmin)<0)
        {
            Qmin = Qab[k];
            move16();
        }
    }
    /*  bring to the same Q */
    FOR ( k=0; k<=HalfLag; k++ )
    {
        X_fx->a_fx[k] = shl(X_fx->a_fx[k], sub(Qmin, Qab[k]));
        move16();     /*  Q(Q+Qab[k]+Qmin-Qab[k])=Q(Q+Qmin) */
        X_fx->b_fx[k] = shl(X_fx->b_fx[k], sub(Qmin, Qab[k]));
        move16();      /*  Q(Q+Qab[k]+Qmin-Qab[k])=Q(Q+Qmin) */
    }

    X_fx->Q = Qmin;
}
/*===================================================================*/
/* FUNCTION      :  poleFilter_setup_fx()                            */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Sets up pole filtering LPC dependent intermediate*/
/*            values to be used by poleFilter                        */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16) lpc[] :  lpc coefficients in Q12                     */
/*   _ (Word16) N     :  lpc order                                   */
/*   _ (Word16) lag      :  in Q0                                    */
/*   _ (Word16 *) S_fx: sin(2pi*n/(4*lag)) table, Q15                */
/*   _ (Word16 *) C_fx: cos(2pi*n/(4*lag)) table, Q15                */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ None                                                          */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*   _ None                                                          */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                                               */
/*===================================================================*/

void poleFilter_setup_fx(const Word16 *LPC, Word16 N, DTFS_STRUCTURE_FX X_fx, Word16 *S_fx, Word16 *C_fx, Word16 *pf_temp1, Word16 *pf_temp2, Word16 *pf_temp, Word16 *pf_n2_temp1)
{
    Word16 temp1, temp2, HalfLag ;
    Word32 sum1_fx, sum2_fx;
    Word32 sum;
    Word16 k, n, n1, n2;
    Word16 exp,tmp;

    HalfLag = s_min(shr(X_fx.lag_fx, 1),X_fx.nH_fx);

    FOR ( k=0; k<=HalfLag; k++ )
    {
        temp2 = k;
        move16();
        /* Calculate sum1 and sum2 */
        sum1_fx = L_deposit_h(4096);      /*  1: Q(12+15+1)  */

        sum2_fx = L_deposit_l(0);
        FOR ( n=0 ; n<N ; n++  )
        {
            sum1_fx = L_mac(sum1_fx, LPC[n], C_fx[(4*temp2)%(4*X_fx.lag_fx)]) ;   /* Q(12+15+1) */
            sum2_fx = L_mac(sum2_fx, LPC[n], S_fx[(4*temp2)%(4*X_fx.lag_fx)]) ;   /* Q(12+15+1) */
            temp2 = add(temp2, k);
        }

        n1 = norm_l(sum1_fx);
        if (sum1_fx==0)
        {
            n1 = 31;
            move16();
        }

        n2 = norm_l(sum2_fx);
        if (sum2_fx==0)
        {
            n2 = 31;
            move16();
        }

        if (sub(n1, n2)<0)
        {
            n2=n1;
            move16();
        }
        n2 = sub(n2, 1);
        temp1 = pf_temp1[k] = round_fx((Word32)L_shl(sum1_fx, n2));   /* Q(12+15+1+n2-16)=Q(12+n2) */
        temp2 = pf_temp2[k] = round_fx((Word32)L_shl(sum2_fx, n2));   /*                  Q(12+n2) */

        /* Calculate the circular convolution */
        sum = L_mac(L_mult(temp1, temp1), temp2, temp2); /*  Q(12+n2+12+n2+1)=Q(25+2*n2) */

        exp = norm_l(sum);
        tmp = extract_h(L_shl(sum,exp));
        exp = sub(sub(30,exp),add(25,shl(n2,1)));
        tmp = div_s(16384,tmp); /* Q(15+exp) */
        sum = tmp;

        pf_n2_temp1[k] = add(n2, exp);
        move16();
        pf_temp[k] = (Word16) sum;
        move16();             /*  Q15+exp */
    }
}




/*=================================================================================*/
/* FUNCTION      :  DTFS_getEngy_band_wb_fx (Word16 lband, Word16 hband)     */
/*---------------------------------------------------------------------------------*/
/* PURPOSE       :  compute the energy of X1.a[k] and X2.b[k]                      */
/*             Get DTFS energy in the specified range from lband to hband.             */
/*             This function is different to "DTFS_getEngy_band" as this can calculate */
/*             lband, hband \in [1,6400] where "DTFS_getEngy_band" only upperlimited to*/
/*             4Khz. Possibility: modify ""DTFS_getEngy_band"" and get rid of this     */
/*             function.                                                               */
/*---------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                              */
/*   _ (struct DTFS_STRUCTURE_FX) X_fx :  a_fx/b_fx in X_fx.Q, lag in Q0           */
/*   _ (Word16) lband:  Q0                                                         */
/*   _ (Word16) hband:  Q0                                                         */
/*---------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                              */
/*   _ (Word32) en_fx :       2*X1.Q+1                                             */
/*---------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                        */
/*                    _ None                                                       */
/*---------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                                      */
/*---------------------------------------------------------------------------------*/
/* CALLED FROM : TX                                                                */
/*=================================================================================*/
Word32 DTFS_getEngy_band_wb_fx(DTFS_STRUCTURE_FX X_fx,Word16 lband, Word16 hband)
{
    Word16 k, lk, hk, HalfLag ;
    Word32 freq_fx, L_lband, L_hband;
    Word32 en_fx=0;

    L_lband = L_mult(lband, X_fx.lag_fx);
    L_hband = L_mult(hband, X_fx.lag_fx);
    HalfLag = shr(sub(X_fx.lag_fx, 1), 1);

    /* get lband and hband */
    FOR (k=1; k<=HalfLag; k++)
    {
        freq_fx = L_mult(k, 12800);

        if (L_sub(freq_fx, L_lband)>=0)
        {
            BREAK;
        }
    }
    lk = k;
    move16();
    FOR (k=1; k<=HalfLag; k++)
    {
        freq_fx = L_mult(k, 12800);
        if (L_sub(freq_fx, L_hband)>=0)
        {
            BREAK;
        }
    }
    hk = k;
    move16();

    FOR (k=lk; k<hk; k++)
    {
        en_fx = L_mac0(en_fx, X_fx.a_fx[k], X_fx.a_fx[k]);          /*  2*X1.Q+1 */
        en_fx = L_mac0(en_fx, X_fx.b_fx[k], X_fx.b_fx[k]);
    }
    en_fx = L_shr(en_fx, 1);                         /*  2*X1.Q+1 */

    if (lband == 0)
    {
        en_fx = L_mac0(en_fx, X_fx.a_fx[0], X_fx.a_fx[0]) ;             /*  2*X1.Q+1 */
    }

    /* IF ((X_fx.lag_fx%2 == 0) && (hband == X_fx.upper_cut_off_freq_fx))  */
    test();
    IF ((s_and(X_fx.lag_fx , 1) == 0)&& (hband == X_fx.upper_cut_off_freq_fx))
    {
        en_fx = L_mac0(en_fx, X_fx.a_fx[k], X_fx.a_fx[k]);
        en_fx = L_mac0(en_fx, X_fx.b_fx[k], X_fx.b_fx[k]);
    }

    return en_fx ;                                     /*  2*X1.Q+1 */
}



