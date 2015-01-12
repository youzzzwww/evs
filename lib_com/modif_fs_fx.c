/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include <assert.h>
#include "rom_enc_fx.h"      /*  prototypes                       */
#include "stl.h"

#include "basop_util.h"

/*-----------------------------------------------------------------*
 * Local functions
 *-----------------------------------------------------------------*/



/*==============================================================================*/
/* FUNCTION      :  modify_Fs_fx ( )                        */
/*------------------------------------------------------------------------------*/
/* PURPOSE       :  Modify sampling freq by interpolation                 */
/*------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                              */
/*  const Word16 sigIn_fx[]     signal to decimate  Q_syn2-1          */
/*      const Word16 lg       length of input                     */
/*      const Word16 fin     frequency of input  Q0                          */
/*    const Word16 fout     frequency of output Q0              */
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                              */
/* Word16 sigOut_fx[]       decimated signal    Q_syn2-1          */
/*------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                            */
/*    Word16 mem_fx[]       filter memory     Q_syn2-1                   */
/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                               */
/*           _ None.                            */
/*------------------------------------------------------------------------------*/
/* CALLED FROM : TX/RX                              */
/*==============================================================================*/
Word16 modify_Fs_fx(            /* o  : length of output    Q0  */
    const Word16 sigIn_fx[],    /* i  : signal to decimate  Q0  */
    Word16 lg,            /* i  : length of input     Q0  */
    const Word32 fin,           /* i  : frequency of input  Q0  */
    Word16 sigOut_fx[],   /* o  : decimated signal    Q0  */
    const Word32 fout,          /* i  : frequency of output Q0  */
    Word16 mem_fx[]       /* i/o: filter memory       Q0  */
    ,const Word16 nblp           /* i  : flag indicating if NB low-pass is applied */
)
{
    Word16 i;

    Word16 lg_out, fac_num, fac_den, filt_len, frac, temp_n, mem_len;
    Word16 num_den;
    Word16 datastep, fracstep;
    Word16 *sigIn_ptr, *sigPtr;
    Word16 signal_tab_fx[3*L_FILT_MAX + L_FRAME48k], *signal_fx, *signal_ana_fx; /* 3* as 2* for memory and 1* for future prediction */
    Word16 A_fx[M+1], r_fx_h[M+1], r_fx_l[M+1];
    Word16 mem_len_ana;
    Word16 plus_sample_in;
    Word16 j;
    Word16 mu_preemph_fx;
    Word16 mem_preemph_fx;
    Word16 Q_r;
    Word16 mem_lev_fx[18];
    Word32 t0, t1, t2, L_tmp;             /* temporary variables                         */
    Word32 LepsP[M+1];

    const Resampling_cfg_fx *cfg_ptr_fx;

    /*-------------------------------------------------------------------*
     * IIR filters for resampling to/from 8 kHz
     *-------------------------------------------------------------------*/

    /*-------------------------------------------------------------------*
     * Find the resampling configuration
     *-------------------------------------------------------------------*/

    /* check if fin and fout are the same */
    IF (L_sub(fin,fout) == 0)
    {
        /* just copy the signal_fx and quit */
        Copy(sigIn_fx, sigOut_fx, lg);

        return lg;
    }
    ELSE
    {
        /* find the resampling configuration in the lookup table */
        cfg_ptr_fx = &resampling_cfg_tbl_fx[0];
        WHILE ( (cfg_ptr_fx->fin_fx != 0) && !(L_sub(cfg_ptr_fx->fin_fx,fin) == 0 && L_sub(cfg_ptr_fx->fout_fx,fout) == 0) )
        {
            test();
            test();
            cfg_ptr_fx++;
        }


        /* find config with NB 4kHz low-pass */
        test();
        test();
        IF ( nblp && (L_sub(fin, 8000) > 0) && (L_sub(fout, 12800) == 0) )
        {
            cfg_ptr_fx++;
            WHILE ( (cfg_ptr_fx->fin_fx != 0) && !( (L_sub(cfg_ptr_fx->fin_fx, fin) == 0) && (L_sub(cfg_ptr_fx->fout_fx, fout) == 0)) )
            {
                test();
                test();
                cfg_ptr_fx++;
            }
        }

        /*-------------------------------------------------------------------*
         * Retrieve and/or calculate the resampling parameters
         *-------------------------------------------------------------------*/
        fac_num = cfg_ptr_fx->fac_num_fx;
        move16();/*Q0*/
        fac_den = cfg_ptr_fx->fac_den_fx;
        move16();

        IF(sub(lg,L_FRAME)>=0)
        {
            lg_out = cfg_ptr_fx->lg_out ;
            move16();
        }
        ELSE
        {
            lg_out = idiv1616(i_mult2(lg, fac_num), fac_den);
        }
        filt_len = cfg_ptr_fx->filt_len_fx;
        move16();
        plus_sample_in = 0;
        move16();/*default, regular delay*/
        frac = 0;
        move16();
        test();
        IF ( (L_sub(fin, 8000) == 0) && (L_sub(fout, 12800) == 0) )
        {
            plus_sample_in = 7;
            move16();
            frac = 4;
            move16();
        }
        mem_len = shl(filt_len,1);
        signal_fx = signal_tab_fx+2*L_FILT_MAX + sub(L_FRAME48k, add(mem_len, lg));
        signal_ana_fx = signal_fx;
        mem_len_ana = mem_len;
        move16();
    }


    /*-------------------------------------------------------------------*
     * Resample
     *-------------------------------------------------------------------*/
    /* append filter memory */
    Copy(mem_fx, signal_fx, mem_len);

    sigPtr = signal_fx + mem_len;
    Copy(sigIn_fx, sigPtr, lg);

    IF(plus_sample_in > 0)
    {
        autocorr_fx( signal_ana_fx+mem_len_ana+lg-LEN_WIN_SSS, 1, r_fx_h, r_fx_l, &Q_r, LEN_WIN_SSS, wind_sss_fx, 0, 0 );


        t1 = L_Comp(r_fx_h[1], r_fx_l[1]);             /* R[1] in Q31      */
        t2 = L_abs(t1);                        /* abs R[1]         */
        t0 = L_deposit_l(0);
        IF (r_fx_h[0] != 0)
        {
            t0 = Div_32(t2, r_fx_h[0], r_fx_l[0]);     /* R[1]/R[0] in Q31 */
        }
        if (t1 < 0)
        {
            t0 = L_negate(t0);                 /* R[1]/R[0]       */
        }



        mu_preemph_fx = extract_h(t0); /*r_fx[1] / r_fx[0]; */
        mem_preemph_fx = signal_ana_fx[mem_len_ana+lg-LEN_WIN_SSS - 1];
        move16();
        preemph_fx(signal_ana_fx+mem_len_ana+lg-LEN_WIN_SSS, mu_preemph_fx, LEN_WIN_SSS, &mem_preemph_fx);


        /* Autocorrelations */
        autocorr_fx( signal_ana_fx+mem_len_ana+lg-LEN_WIN_SSS, M, r_fx_h, r_fx_l, &Q_r,
                     LEN_WIN_SSS, wind_sss_fx, 0, 0 );

        /* Lag windowing */
        lag_wind( r_fx_h, r_fx_l, M, fin, LAGW_STRONG );

        /* Levinson-Durbin */
        set16_fx(mem_lev_fx, 0, 18 );
        E_LPC_lev_dur(r_fx_h, r_fx_l, A_fx, LepsP, M, NULL);

        Copy_Scale_sig( A_fx, A_fx, M+1, sub(norm_s(A_fx[0]),2) );

        FOR (i=0; i<plus_sample_in; i++)
        {
            sigPtr = signal_fx+lg+mem_len+i;
            move16();  /*+i*/
            L_tmp = syn_kern_16(0, A_fx, sigPtr);
            L_tmp = L_shl(L_tmp, 3);
            *sigPtr = round_fx(L_tmp);  /* AZ ringing padding */
        }
        mem_preemph_fx = signal_fx[mem_len+lg-LEN_WIN_SSS - 1];
        move16();
        deemph_fx(signal_fx+mem_len+lg-LEN_WIN_SSS, mu_preemph_fx, LEN_WIN_SSS+plus_sample_in, &mem_preemph_fx);
    }
    /* interpolation */

    datastep = shr(div_s(shl(fac_den,8), shl(fac_num,11)),12);
    /* equivalent to 'datastep = fac_den % fac_num' */
    temp_n = i_mult2(datastep,fac_num);                            /*Q0*/
    fracstep = sub(fac_den,temp_n);

    sigIn_ptr = signal_fx + add(filt_len,  plus_sample_in);
    FOR(i=0; i<lg_out; i++)
    {
        sigOut_fx[i] = round_fx(Interpol_lc_fx( sigIn_ptr, cfg_ptr_fx->filter_fx, frac, fac_num, filt_len ));

        frac = add(frac,fracstep);

        j = sub(fac_num, frac);
        if (j < 0)
        {
            frac = sub(frac,fac_num);
        }
        sigIn_ptr += add(lshr(j, 15), datastep);
    }
    /* rescaling */
    test();
    IF ((sub(fac_num,fac_den) > 0) == ((cfg_ptr_fx->flags_fx & RS_INV_FAC) != 0))
    {
        IF(sub(fac_num, fac_den) < 0)
        {
            num_den = div_s(fac_num,fac_den);/*Q15*/
            test();
            IF( L_sub(fin, 16000) > 0 && sub(lg_out, 512) == 0 )
            {

                FOR( i=0; i<lg_out; i++ )
                {
                    sigOut_fx[i] = round_fx(L_shl(L_mult(sigOut_fx[i], num_den), 1));/*Q0*/
                }

            }
            ELSE
            {
                test();
                test();
                test();
                if( L_sub(fin, 16000) > 0 && ( sub(lg_out, L_FRAME) == 0 || sub(lg_out, L_FRAME16k) == 0 || sub(lg_out, 512) == 0) )
                {
                    num_den = shl(num_den, 1);
                }
                FOR( i=0; i<lg_out; i++ )
                {
                    sigOut_fx[i] = mult_r(sigOut_fx[i],num_den);/*Q0*/  move16();
                }
            }
        }
        ELSE
        {
            IF(sub(fac_num,8)==0)
            {
                num_den = 26214;
                FOR( i=0; i<lg_out; i++ )
                {
                    sigOut_fx[i] = mult_r(sigOut_fx[i],num_den);/*Q-1*/ move16();
                }
            }
            ELSE
            {
                num_den = div_s(sub(fac_num,fac_den),fac_den);/*Q15*/
                FOR( i=0; i<lg_out; i++ )
                {
                    sigOut_fx[i] = round_fx(L_mac(L_deposit_h(sigOut_fx[i]),sigOut_fx[i],num_den));/*Q0*/
                }
            }
        }
    }
    ELSE IF((sub(fac_num,fac_den) < 0) && ((cfg_ptr_fx->flags_fx & RS_INV_FAC) != 0))
    {
        FOR( i=0; i<lg_out; i++ )
        {
            sigOut_fx[i] = mult_r(sigOut_fx[i],16384);
            move16();/*Q-1*/
        }
    }
    /* update the filter memory */
    sigPtr = signal_fx+lg;
    Copy(sigPtr, mem_fx, mem_len);

    return lg_out;
}

Word16 modify_Fs_intcub3m_sup_fx(       /* o  : length of output    */
    const Word16 sigIn[],          /* i  : signal to decimate with memory of 2 samples (indexes -2 & -1) */
    const Word16 lg,               /* i  : length of input (suppose that lg is such that lg_out is integer, ex multiple of 5 in case of 16kHz to 12.8 kHz) */
    const Word32   fin,              /* i  : frequency of input  */
    Word16 sigOut[],         /* o  : decimated signal    */
    const Word32   fout,             /* i  : frequency of output */
    Word16 *delayout         /* o  : delay of output */
)
{
    Word16 i, k, i1, i2, k1, k2, k3, kk, cind;
    Word16 lg_out, fk1, k2d, k3d;
    Word16 cc[4][4];
    const Word16 (*cu)[3] = 0;
    Word16 *sigin_sr, *sigOutptr, *cptr;
    const Word16 *uptr, *ctptr;
    Word16 *sigin_sr_tab;
    Word16 lim, inc, lim2, lim3;
    Word32 vv32;
#define QSR  2  /* right shift to avoid overflow, 2 is OK */

    k = 0;
    move16();  /* to avoid compilation warnings */
    inc = 0;
    move16();    /* to avoid compilation warnings */

    /*-------------------------------------------------------------------*
     * Find the resampling configuration
     *-------------------------------------------------------------------*/

    /* check if fin and fout are the same */
    IF (L_sub(fin,fout) == 0)
    {
        /* just copy the signal_fx and quit */
        Copy(sigIn, sigOut, lg);
        *delayout = 0;
        move16();

        return lg;
    }
    ELSE
    {
        sigin_sr_tab = (Word16*)calloc(lg+2, sizeof(*sigin_sr)); /*shift right*/
        sigin_sr = sigin_sr_tab+2;
        FOR(i = -2; i < lg; i++)
        {
            sigin_sr[i] = shr(sigIn[i], QSR);
            move16();  /* shift right : Q0 -> Q(-QSR) */
        }

        /* length of the interpolated signal */
        /*lg_out = (short)(lg * fout / fin); */

        /* cc[x][3]*s*s*s + cc[x][2]*s*s + cc[x][1]*s + cc[x][0]; indexes relatives of s : -1 0 1 2 */
        /* d : cc[x][0] = s[0] */
        /* b : cc[x][2] =(s[-1]+s[1])/2-s[0] */
        /* a : cc[x][3] = (s[-1]+s[2]-s[0]-s[1]-4*cc[x][2]) / 6 */
        /* c : cc[x][1] = s[1]-s[0]-cc[x][3]-cc[x][2] */

        /* coef inits using memory (indexes < 0) */
        /* cc[2][] : indexes -2 -1 0 1 */
        cptr = &(cc[2][0]);
        cptr[0] = mult_r(sigin_sr[-1], 10923);
        move16(); /* sigIn[-1]/3    */
        cptr[2] = sub(shr(add(sigin_sr[-2], sigin_sr[0]), 1), sigin_sr[-1]);
        move16();                        /* (sigIn[-2]+sigIn[0])/2-sigIn[-1]; */
        cptr[3] = sub(mult_r(sub(add(sigin_sr[-2], sigin_sr[1]), add(sigin_sr[-1], sigin_sr[0])), 5461), mult_r(cptr[2],21845));
        move16(); /*(sigIn[-2]+sigIn[1]-sigIn[-1]-sigIn[0]) / 6 - 4/6*cc[2][2]);*/
        cptr[1] = sub(sub(sigin_sr[0], sigin_sr[-1]), add(cptr[3], cptr[2]));
        move16();

        /* cc[3][] : indexes -1 0 1 2 */
        cptr = &(cc[3][0]);
        cptr[0] = mult_r(sigin_sr[0], 10923);
        move16();/* sigIn[-1]/3    */
        cptr[2] = sub(shr(add(sigin_sr[-1], sigin_sr[1]), 1), sigin_sr[0]);
        move16();                        /* (sigIn[-1]+sigIn[1])/2-sigIn[0]; */
        cptr[3] = sub(mult_r(sub(add(sigin_sr[-1], sigin_sr[2]), add(sigin_sr[0], sigin_sr[1])), 5461), mult_r(cptr[2],21845));
        move16(); /*(sigIn[-2]+sigIn[1]-sigIn[-1]-sigIn[0]) / 6 - 4/6*cc[2][2]);*/
        cptr[1] = sub(sub(sigin_sr[1], sigin_sr[0]), add(cptr[3], cptr[2]));
        move16();

        sigOutptr = sigOut;
        cind = -1;
        move16();
        move16(); /* for the move */
        IF( L_sub(fin, 12800) == 0 )
        {
            if( L_sub(fout, 8000) == 0 )
            {
                cind = 0;
                move16();
            }
            if( L_sub(fout, 16000) == 0 )
            {
                cind = 1;
                move16();
            }
            if( L_sub(fout, 32000) == 0 )
            {
                cind = 2;
                move16();
            }
            if( L_sub(fout, 48000) == 0 )
            {
                cind = 3;
                move16();
            }
        }
        IF( L_sub(fin, 16000) == 0)
        {
            if( L_sub(fout, 12800) == 0 )
            {
                cind = 4;
                move16();
            }
            if( L_sub(fout, 32000) == 0 )
            {
                cind = 5;
                move16();
            }
            if( L_sub(fout, 48000) == 0 )
            {
                cind = 6;
                move16();
            }

        }
        ctptr = &(ct2_fx[cind][0]);
        lg_out = mult_r(shl(lg,2),ctptr[13]);
        *delayout = ctptr[9];
        move16();

        if( sub(ctptr[12], 15) == 0 )
        {
            cu = cu15_fx;
            move16();/*pointer*/
        }

        if( sub(ctptr[12],4) == 0 )
        {
            cu = cu4_fx;
            move16();/*pointer*/
        }

        k2d = sub(ctptr[12], 1); /* shift of index in cu with respect to the next sample (ex 1.25 -> 0.25 ) */
        fk1 = shl(k2d, 1);
        k3d = sub(fk1, 1); /* to compurte index in cu with respect to the last sample with - sign (ex 1.25 -> -0.75 ) */

        kk = 0;
        move16();
        i = 0;
        move16();

        lim =  sub(lg, ctptr[11]);
        lim2 = sub(lg, 3);
        lim3 = ctptr[10];
        move16();
        WHILE(sub(i, lim2) < 0)
        {
            if(sub(i, lim) >= 0)
            {
                lim3 = sub(ctptr[11], 3); /* last, incomplete period*/
            }
            *sigOutptr++ = sigIn[i];
            move16();
            FOR(k = 0; k < lim3; k++)
            {
                cptr = &(cc[kk][0]);
                cptr[0] = mult_r(sigin_sr[i+1], 10923);
                move16();/* sigIn[-1]/3    */
                cptr[2] = sub(shr(add(sigin_sr[i], sigin_sr[i+2]), 1), sigin_sr[i+1]);
                move16();                        /* (sigIn[-1]+sigIn[1])/2-sigIn[0]; */
                cptr[3] = sub(mult_r(sub(add(sigin_sr[i], sigin_sr[i+3]), add(sigin_sr[i+1], sigin_sr[i+2])), 5461), mult_r(cptr[2],21845));
                move16(); /*(sigIn[-1]+sigIn[1]-sigIn[0]-sigIn[1]) / 6 - 4/6*cc[3][2]);*/
                cptr[1] = sub(sub(sigin_sr[i+2], sigin_sr[i+1]), add(cptr[3], cptr[2]));
                move16();
                i = add(i,1);

                i2 = sub(kk, 2);
                i1 = sub(kk, 1);
                if( i1 < 0 )
                {
                    i1 = add(i1, 4);
                }

                if( i2 < 0 )
                {
                    i2 = add(i2, 4);
                }
                inc = ctptr[8];
                move16();
                FOR(k1 = ctptr[k]; k1 < fk1; k1 += inc)
                {
                    k2 = sub(k1, k2d);
                    k3 = sub(k3d, k1);
                    cptr = &(cc[i2][0]);
                    uptr = &(cu[k1][0]);
                    vv32 = L_mult(     8192,    *cptr++);
                    vv32 = L_mac(vv32, *uptr++, *cptr++);
                    vv32 = L_mac(vv32, *uptr++, *cptr++);
                    vv32 = L_mac(vv32, *uptr++, *cptr++); /* Q13*Q(-QSR) -> Q(13-QSR+1) 32 bits*/
                    cptr = &(cc[i1][0]);
                    uptr = &(cu[k2][0]);
                    vv32 = L_mac(vv32, 8192,    *cptr++);
                    vv32 = L_mac(vv32, *uptr++, *cptr++);
                    vv32 = L_mac(vv32, *uptr++, *cptr++);
                    vv32 = L_mac(vv32, *uptr++, *cptr++);
                    cptr = &(cc[kk][0]);
                    uptr = &(cu[k3][0]);
                    vv32 = L_mac(vv32, 8192,    *cptr++);
                    vv32 = L_msu(vv32, *uptr++, *cptr++);
                    vv32 = L_mac(vv32, *uptr++, *cptr++);
                    vv32 = L_msu(vv32, *uptr++, *cptr++);
                    vv32 = L_shl(vv32, (QSR+2)); /* Q(13-QSR+1) -> Q16 */
                    *sigOutptr++ = round_fx(vv32); /* Q16 -> Q0*/
                }

                kk = add(kk, 1);
                if( sub(kk, 4) == 0 )
                {
                    kk = 0;
                    move16();
                }
            }
        }


        kk = sub(kk, 1);
        if( kk < 0 )
        {
            kk = 3;
            move16();
        }

        if( sub(ctptr[10], 1) == 0 )
        {
            *sigOutptr++ = sigIn[i];
            move16();
        }

        FOR(k1 = ctptr[k]; k1 < fk1; k1 += inc)
        {
            k2 = sub(k1, k2d);

            cptr = &(cc[kk][0]);
            uptr = &(cu[k2][0]);
            vv32 = L_mult(     8192,    *cptr++);
            vv32 = L_mac(vv32, *uptr++, *cptr++);
            vv32 = L_mac(vv32, *uptr++, *cptr++);
            vv32 = L_mac(vv32, *uptr++, *cptr++); /* Q13*Q(-QSR) -> Q(13-QSR+1) 32 bits*/
            vv32 = L_shl(vv32, (QSR+2)); /* Q(13-QSR+1) -> Q16 */
            *sigOutptr++ = i_mult(round_fx(vv32), 3);
            move16();/* Q16 -> Q0*/
        }

        if( sub(ctptr[10], 3) < 0 )
        {
            *sigOutptr++ = sigIn[add(i, 1)];
            move16();
        }

        FOR( k1 = ctptr[add(k, 1)]; k1 < fk1; k1 += inc )
        {
            cptr = &(cc[kk][0]);
            uptr = &(cu[k1][0]);
            vv32 = L_mult(     8192,    *cptr++);
            vv32 = L_mac(vv32, *uptr++, *cptr++);
            vv32 = L_mac(vv32, *uptr++, *cptr++);
            vv32 = L_mac(vv32, *uptr++, *cptr++); /* Q13*Q(-QSR) -> Q(13-QSR+1) 32 bits*/
            vv32 = L_shl(vv32, (QSR+2)); /* Q(13-QSR+1) -> Q16 */
            *sigOutptr++ = i_mult(round_fx(vv32), 3);
            move16();/* Q16 -> Q0*/
        }

        if( sub(ctptr[10], 1) == 0 )
        {
            *sigOutptr = sigIn[add(i, 2)];
            move16();
        }
    }

    free(sigin_sr_tab);

    return lg_out;
}

/*====================================================================*/
/* FUNCTION      :  Decimate_allpass_steep_fx1 ()                     */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  decimation by a factor 2               */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (const Word16 *) in_fx :  input speech, Q0                    */
/*   _ (Word16 []) state_fx: Stateinfo, Q0                           */
/*                     Size: 2*ALLPASSSECTIONS_STEEP+1               */
/*   _ (Word16) N: Number of Input samples                           */
/*   _ (Word16 []) out_fx    :Output arry of size N/2 Q0        */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*   _ (Word16 []) out_fx : output-signal, Q0                        */
/*   _ (Word16 []) state_fx:gets updated, Q0                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*===================================================================*/


void Decimate_allpass_steep_fx( const Word16 *in_fx,
                                Word16 state_fx[],           /* array of size: 2*ALLPASSSECTIONS_STEEP+1 */
                                Word16  N,                 /* number of input samples */
                                Word16 out_fx[])             /* array of size N/2 */
{

    Word16  k;
    Word16 temp[ ALLPASSSECTIONS_STEEP ];
    Word32 Lacc, Lacc1;
    Word16 temp1, temp2;
    Word16 sum = 0;
    move16();

    /*upper allpass filter chain  */


    FOR ( k = 0; k < N/2; k++ )
    {

        Lacc  = L_deposit_h( state_fx[0] );                     /* Q(16+x)   */
        Lacc  = L_mac( Lacc, AP1_STEEP_FX[0], in_fx[2*k] );     /* Q(16+x)   */
        Lacc1 = L_deposit_h( in_fx[2*k] );                      /* Q16+Qx    */
        temp1 = extract_h( Lacc );                              /* Qx        */
        Lacc1 = L_msu( Lacc1, AP1_STEEP_FX[0], temp1 );         /* Q16+Qx    */

        state_fx[0] = extract_h( Lacc1 );                       /* Qx        */
        temp[0] = temp1;
        move16();

        Lacc1 = L_deposit_h( state_fx[1] );                     /* Q16+Qx    */
        Lacc1 = ( L_mac( Lacc1, AP1_STEEP_FX[1], temp1 ));      /* Q16+Qx    */

        temp2 = extract_h( Lacc1 );                             /* Qx        */
        Lacc = L_msu( Lacc, AP1_STEEP_FX[1], temp2 );           /* Q16+Qx    */
        state_fx[1] = extract_h(Lacc);                          /* Qx        */
        temp[1] = temp2;
        move16();


        Lacc = L_deposit_h( state_fx[ ALLPASSSECTIONS_STEEP-1 ] );                                              /* Q(16+x)              */
        out_fx[ k ] = extract_h( L_mac( Lacc, AP1_STEEP_FX[ ALLPASSSECTIONS_STEEP-1 ], temp2 ));                /* Qx format            */
        state_fx[ ALLPASSSECTIONS_STEEP-1 ] = extract_h( L_msu ( Lacc1, AP1_STEEP_FX[ ALLPASSSECTIONS_STEEP-1 ], out_fx[k] )); /* Qx    */

    }

    /* lower allpass filter chain  */

    Lacc  = L_deposit_h( state_fx[ ALLPASSSECTIONS_STEEP ] );               /*  Q(16+x)     */
    Lacc  = L_mac( Lacc, AP2_STEEP_FX[0], state_fx[2*ALLPASSSECTIONS_STEEP] );  /*Q(16+x)   */
    Lacc1 = L_deposit_h( state_fx[2*ALLPASSSECTIONS_STEEP] );               /*  Q(16+x)     */
    temp1 = extract_h( Lacc );                                              /*  Qx          */
    Lacc1 = L_msu( Lacc1, AP2_STEEP_FX[0], temp1 );                         /*  Q(16+x)     */

    state_fx[ ALLPASSSECTIONS_STEEP ] = extract_h( Lacc1 );
    temp[0] = temp1;
    move16();

    Lacc1 = L_deposit_h( state_fx[ ALLPASSSECTIONS_STEEP+1 ] );            /*   Q(16+x) */
    Lacc1 = L_mac( Lacc1, AP2_STEEP_FX[1], temp1 );                         /*  Q(16+x) */
    temp2 = extract_h( Lacc1 );                                             /*  Qx      */
    temp[1] = temp2;
    move16();
    Lacc = L_msu( Lacc, AP2_STEEP_FX[1], temp2 );                           /*  Q(16+x) */
    state_fx[ALLPASSSECTIONS_STEEP+1] = extract_h( Lacc );                  /*  Qx      */


    Lacc = L_deposit_h( state_fx[2*ALLPASSSECTIONS_STEEP-1] );               /* Q(16+x)  */
    Lacc = L_mac( Lacc, AP2_STEEP_FX[ALLPASSSECTIONS_STEEP-1], temp2 );      /* Q(16+x) temp[ALLPASSSECTIONS_STEEP-1] */
    temp[2] = extract_h( Lacc );                                             /* temp[2] in Qx  */
    Lacc1 = L_msu( Lacc1, AP2_STEEP_FX[ALLPASSSECTIONS_STEEP-1], temp[2] );    /*   Q(16+x) */
    state_fx[ 2*ALLPASSSECTIONS_STEEP-1 ] = extract_h( Lacc1 );                  /*   Qx      */

    sum = mult_r( out_fx[0], 16384 ); /* Qx */
    out_fx[0] = add( sum, mult_r(temp[ALLPASSSECTIONS_STEEP-1], 16384 ));   /*    Qx  */ move16();


    FOR ( k = 1; k < N/2; k++)
    {


        Lacc  = L_deposit_h( state_fx[ ALLPASSSECTIONS_STEEP ] );               /* Q(16+x)           */
        Lacc  = L_mac(Lacc, AP2_STEEP_FX[0], in_fx[2*k-1] );                    /* Q(16+x):temp[0]   */
        Lacc1 = L_deposit_h( in_fx[ 2*k-1 ] );                                  /* Q(16+x)           */
        temp1 = extract_h( Lacc );                                              /* Qx                */
        Lacc1 = L_msu( Lacc1, AP2_STEEP_FX[0], temp1 );                         /* Q(16+x)           */

        state_fx[ALLPASSSECTIONS_STEEP] = extract_h( Lacc1 );                   /* Qx */
        temp[0] = temp1;
        move16();


        Lacc1 = L_deposit_h( state_fx[ ALLPASSSECTIONS_STEEP+1 ] );             /* Q(16+x)  */
        Lacc1 = L_mac(Lacc1,AP2_STEEP_FX[1],temp1);                             /* Q(16+x)  */
        temp2 = extract_h( Lacc1 );                                             /* Qx       */
        temp[1] = temp2;
        move16();
        Lacc = L_msu(Lacc,AP2_STEEP_FX[1],temp2);                               /* Q(16+x)  */
        state_fx[ALLPASSSECTIONS_STEEP+1]= extract_h(Lacc);                     /* Qx       */


        Lacc  = L_deposit_h( state_fx[2*ALLPASSSECTIONS_STEEP-1] );                 /* Q(16+x) */
        Lacc = L_mac( Lacc, AP2_STEEP_FX[ALLPASSSECTIONS_STEEP-1], temp[1] );       /* Q(16+x) temp[ALLPASSSECTIONS_STEEP-1] */
        temp[2] = extract_h( Lacc );    /*temp[2] in Qx  */
        Lacc1 = L_msu( Lacc1, AP2_STEEP_FX[ALLPASSSECTIONS_STEEP-1], temp[2] );     /* Q(16+x) */
        state_fx[2*ALLPASSSECTIONS_STEEP-1] = extract_h( Lacc1 );                   /* Qx  */



        sum = mult_r( out_fx[k], 16384 );                                         /* Qx */
        out_fx[k] = add( sum, mult_r( temp[ALLPASSSECTIONS_STEEP-1], 16384 ) );
        move16();   /* Qx  */

    }

    /* z^(-1) */

    state_fx[ 2*ALLPASSSECTIONS_STEEP ] = in_fx[ N-1 ];
    move16();               /* Qx */

}



void Interpolate_allpass_steep_fx(const Word16 *in_fx,
                                  Word16 state_fx[],           /* array of size: 2*ALLPASSSECTIONS_STEEP+1 */
                                  Word16  N,                   /* number of input samples */
                                  Word16 out_fx[])             /* array of size 2*N */


{

    Word16 k;
    Word32 Lacc=0, Lacc1=0;
    Word16 temp1, temp2;

    /*** State in Q0,in_fx Q0, AP1_STEEP in Q15 AP2_STEEP in Q15  OP in Q0 ************/
    /*upper allpass filter chain     */

    FOR (k=0; k<N; k++)
    {

        Lacc = L_deposit_h( state_fx[0] );               /* Q(16+x) */
        Lacc = L_mac( Lacc, AP2_STEEP_FX[0], in_fx[k] );  /* Q(16+x):temp[0] */

        Lacc1 = L_deposit_h( in_fx[k] );             /* Q(16+x) */
        temp1 = round_fx( Lacc );                   /* Qx */
        Lacc1 = L_msu( Lacc1, AP2_STEEP_FX[0], temp1 ); /* Q(16+x)  */

        state_fx[0] = round_fx( Lacc1 );

        Lacc1 = L_deposit_h( state_fx[1] );            /* Q(16+x) */
        Lacc1 = ( L_mac(Lacc1, AP2_STEEP_FX[1], temp1 ));  /* Q(16+x):temp[1] */

        Lacc = L_deposit_h( temp1 );

        temp2 = round_fx( Lacc1 );                    /* Qx */
        Lacc = L_msu( Lacc, AP2_STEEP_FX[1], temp2 );    /* Q(16+x) */
        state_fx[1] = round_fx( Lacc );                /* Qx */

        Lacc1 = L_deposit_h( temp2 );
        Lacc = L_deposit_h( state_fx[ALLPASSSECTIONS_STEEP-1] );            /* Q(16+x) */
        out_fx[2*k+1] = round_fx( L_mac( Lacc, AP2_STEEP_FX[ALLPASSSECTIONS_STEEP-1], temp2 )); /* Qx format */
        state_fx[ALLPASSSECTIONS_STEEP-1] = round_fx( L_msu(Lacc1, AP2_STEEP_FX[ALLPASSSECTIONS_STEEP-1], out_fx[2*k+1] ));/* Qx  */

    }

    /*  lower allpass filter chain    */

    FOR ( k = 0; k < N; k++)
    {
        Lacc = L_deposit_h( state_fx[ ALLPASSSECTIONS_STEEP ] );               /* Q(16+x) */
        Lacc = L_mac( Lacc, AP1_STEEP_FX[0], in_fx[k] );                       /* Q(16+x):temp[0] */

        Lacc1 = L_deposit_h( in_fx[k] );             /* Q(16+x) */
        temp1 = round_fx( Lacc );                   /* Qx */
        Lacc1 = L_msu( Lacc1, AP1_STEEP_FX[0], temp1 ); /* Q(16+x)  */

        state_fx[ ALLPASSSECTIONS_STEEP ] = round_fx( Lacc1 );

        Lacc1 = L_deposit_h( state_fx[ ALLPASSSECTIONS_STEEP+1 ] );            /* Q(16+x) */
        Lacc1 = L_mac( Lacc1, AP1_STEEP_FX[1], temp1 );  /* Q(16+x):temp[1] */

        temp2 = round_fx( Lacc1 );                    /* Qx */
        Lacc = L_deposit_h( temp1 );
        Lacc = L_msu( Lacc, AP1_STEEP_FX[ 1 ], temp2 );    /* Q(16+x) */
        state_fx[ ALLPASSSECTIONS_STEEP+1 ] = round_fx( Lacc );                /* Qx */

        Lacc = L_deposit_h( state_fx[ 2*ALLPASSSECTIONS_STEEP-1 ] );            /* Q(16+x) */
        Lacc1 = L_deposit_h( temp2 );
        out_fx[ 2*k ] = round_fx( L_mac( Lacc, AP1_STEEP_FX[ ALLPASSSECTIONS_STEEP-1 ], temp2 )); /* Qx format */
        state_fx[ 2*ALLPASSSECTIONS_STEEP-1 ] = round_fx( L_msu(Lacc1, AP1_STEEP_FX[ ALLPASSSECTIONS_STEEP-1 ], out_fx[ 2*k ] ));/* Qx */

    }

    return;
}


/*-------------------------------------------------------------------*
  * interpolate_3_over_2_allpass_fx()
  *
  * Interpolate 3/2 using allpass iir polyphase filter. Delay 4 samples @48k
  *-------------------------------------------------------------------*/

void interpolate_3_over_2_allpass_fx(
    const Word16 *input_fx,           /* i  : input signal            */ /* Q_input */
    const Word16 len,                 /* i  : number of input samples */
    Word16 *out_fx,             /* o  : output signal           */ /* Q_input */
    Word16 *mem_fx,             /* i/o: memory                  */ /* Q_input */
    const Word16 *filt_coeff_fx       /* i  : filter coefficients     */ /* Q15*/
)
{
    /* mem of current frame would be stored in Qinput, so the next call to this function shoulf have Q_mem parameter set to prev_Q_input */
    Word16 i, loop_len;
    Word16 input_fx_temp[L_FRAME32k]; /* Limiting length of input signal to a max of L_FRAME32k samples */
    Word16 Vu[2], Vm[2], Vl[2];    /* Outputs of three cascaded allpass stages (upper, middle, and lower) */ /* will be in Q_current */
    Word16 out1_buff[L_FRAME48k*2];
    Word16 *out1;
    Word16 mem_temp;

    Copy(input_fx, input_fx_temp, len);
    out1 = out1_buff;

    FOR (i = 0; i < len; i++ )
    {
        /* Upper branch */
        /*Vu[0] = mem[0] + filt_coeff[0] * ( input_fx_temp[i] - mem[1] );
        Vu[1] = mem[1] + filt_coeff[1] * ( Vu[0] - mem[2] );
        mem[3] = mem[2] + filt_coeff[2] * ( Vu[1] - mem[3] );*/

        Vu[0] = add(mem_fx[0], mult_r(filt_coeff_fx[0], sub(input_fx_temp[i], mem_fx[1])));
        move16();/* all Vu's in : Q_current*/
        Vu[1] = add(mem_fx[1], mult_r(filt_coeff_fx[1], sub(Vu[0], mem_fx[2])));
        move16();
        mem_fx[3] = add(mem_fx[2], mult_r(filt_coeff_fx[2], sub(Vu[1], mem_fx[3])));
        move16();


        mem_fx[1] = Vu[0];
        move16();
        mem_fx[2] = Vu[1];
        move16();
        *out1++ = mem_fx[3];
        move16();

        /* Middle branch */
        /* Vm[0] = mem[0] + filt_coeff[3] * (input[i]-mem[4]);
        Vm[1] = mem[4] + filt_coeff[4] * (Vm[0]-mem[5]);
        mem[6] = mem[5] + filt_coeff[5] * (Vm[1]-mem[6]); */
        Vm[0] = add(mem_fx[0], mult_r(filt_coeff_fx[3], sub(input_fx_temp[i], mem_fx[4])));
        move16();
        Vm[1] = add(mem_fx[4], mult_r(filt_coeff_fx[4], sub(Vm[0], mem_fx[5])));
        move16();
        mem_fx[6] = add(mem_fx[5], mult_r(filt_coeff_fx[5], sub(Vm[1], mem_fx[6])));
        move16();

        mem_fx[4] = Vm[0];
        move16();
        mem_fx[5] = Vm[1];
        move16();
        *out1++ = mem_fx[6];
        move16();

        /* Lower branch */
        /* Vl[0] = mem[0] + filt_coeff[6] * (input[i]-mem[7]);
        Vl[1] = mem[7] + filt_coeff[7] * (Vl[0]-mem[8]);
        mem[9] = mem[8] + filt_coeff[8] * (Vl[1]-mem[9]); */
        Vl[0] = add(mem_fx[0], mult_r(filt_coeff_fx[6], sub(input_fx_temp[i], mem_fx[7])));
        move16();
        Vl[1] = add(mem_fx[7], mult_r(filt_coeff_fx[7], sub(Vl[0], mem_fx[8])));
        move16();
        mem_fx[9] = add(mem_fx[8], mult_r(filt_coeff_fx[8], sub(Vl[1], mem_fx[9])));
        move16();

        mem_fx[0] = input_fx_temp[i];
        move16();
        mem_fx[7] = Vl[0];
        move16();
        mem_fx[8] = Vl[1];
        move16();
        *out1++ = mem_fx[9];
        move16();
    }

    /* loop_len = len*3/2 */
    loop_len = shr(i_mult(len,3),1);

    /*decimate by 2 and LPF*/
    FOR(i = 0; i < loop_len; i++)
    {
        mem_temp = out1_buff[shl(i,1)];
        move16();
        out_fx[i] = add( mult_r( FL2WORD16( 0.0473147f),add(mem_temp,mem_fx[10]) ), mult_r( FL2WORD16(-0.151521f),add(mem_fx[11],mem_fx[14]) ) );
        out_fx[i] = add( out_fx[i], mult_r( FL2WORD16(0.614152f),add(mem_fx[12],mem_fx[13]) ) );
        mem_fx[10] = mem_fx[11];
        move16();
        mem_fx[11] = mem_fx[12];
        move16();
        mem_fx[12] = mem_fx[13];
        move16();
        mem_fx[13] = mem_fx[14];
        move16();
        mem_fx[14] = mem_temp;
        move16();
    }

    return;
}


/*-------------------------------------------------------------------*
  * interpolate_3_over_1_allpass_fx()
  *
  * Interpolate 3/1 using allpass iir polyphase filter. Delay 4 samples @48k
  *-------------------------------------------------------------------*/

void interpolate_3_over_1_allpass_fx(
    const Word16 *input_fx,           /* i  : input signal            */ /* Q_input */
    const Word16 len,                 /* i  : number of input samples */
    Word16 *out_fx,             /* o  : output signal           */ /* Q_input */
    Word16 *mem_fx,             /* i/o: memory                  */ /* Q_input */
    const Word16 *filt_coeff_fx       /* i  : filter coefficients     */ /* Q15*/
)
{
    /* mem of current frame would be stored in Qinput, so the next call to this function shoulf have Q_mem parameter set to prev_Q_input */
    Word16 i;
    Word16 Vu[2], Vm[2], Vl[2];    /* Outputs of three cascaded allpass stages (upper, middle, and lower) */
    Word16 *out1;
    Word16 mem_temp;

    out1 = &out_fx[0];

    FOR (i = 0; i < len; i++ )
    {
        /* Upper branch */
        /*Vu[0] = mem[0] + filt_coeff[0] * ( input_fx_temp[i] - mem[1] );
        Vu[1] = mem[1] + filt_coeff[1] * ( Vu[0] - mem[2] );
        mem[3] = mem[2] + filt_coeff[2] * ( Vu[1] - mem[3] );*/

        Vu[0] = add(mem_fx[0], mult_r(filt_coeff_fx[0], sub(input_fx[i], mem_fx[1])));
        move16();/* all Vu's in : Q_current*/
        Vu[1] = add(mem_fx[1], mult_r(filt_coeff_fx[1], sub(Vu[0], mem_fx[2])));
        move16();
        mem_fx[3] = add(mem_fx[2], mult_r(filt_coeff_fx[2], sub(Vu[1], mem_fx[3])));
        move16();


        mem_fx[1] = Vu[0];
        move16();
        mem_fx[2] = Vu[1];
        move16();
        *out1++ = mem_fx[3];
        move16();

        /* Middle branch */
        /* Vm[0] = mem[0] + filt_coeff[3] * (input[i]-mem[4]);
        Vm[1] = mem[4] + filt_coeff[4] * (Vm[0]-mem[5]);
        mem[6] = mem[5] + filt_coeff[5] * (Vm[1]-mem[6]); */
        Vm[0] = add(mem_fx[0], mult_r(filt_coeff_fx[3], sub(input_fx[i], mem_fx[4])));
        move16();
        Vm[1] = add(mem_fx[4], mult_r(filt_coeff_fx[4], sub(Vm[0], mem_fx[5])));
        move16();
        mem_fx[6] = add(mem_fx[5], mult_r(filt_coeff_fx[5], sub(Vm[1], mem_fx[6])));
        move16();

        mem_fx[4] = Vm[0];
        move16();
        mem_fx[5] = Vm[1];
        move16();
        *out1++ = mem_fx[6];
        move16();

        /* Lower branch */
        /* Vl[0] = mem[0] + filt_coeff[6] * (input[i]-mem[7]);
        Vl[1] = mem[7] + filt_coeff[7] * (Vl[0]-mem[8]);
        mem[9] = mem[8] + filt_coeff[8] * (Vl[1]-mem[9]); */
        Vl[0] = add(mem_fx[0], mult_r(filt_coeff_fx[6], sub(input_fx[i], mem_fx[7])));
        move16();
        Vl[1] = add(mem_fx[7], mult_r(filt_coeff_fx[7], sub(Vl[0], mem_fx[8])));
        move16();
        mem_fx[9] = add(mem_fx[8], mult_r(filt_coeff_fx[8], sub(Vl[1], mem_fx[9])));
        move16();

        mem_fx[0] = input_fx[i];
        move16();
        mem_fx[7] = Vl[0];
        move16();
        mem_fx[8] = Vl[1];
        move16();
        *out1++ = mem_fx[9];
        move16();
    }
    /*LPF*/
    FOR(i = 0; i < len*3; i++)
    {
        mem_temp = out_fx[i];
        move16();
        out_fx[i] = sub(mult_r(FL2WORD16(0.57276865021499168f), add(mem_fx[12], mem_fx[11])), mult_r(FL2WORD16(0.074004974641176793f),add(mem_temp,mem_fx[10])));
        mem_fx[10] = mem_fx[11];
        move16();
        mem_fx[11] = mem_fx[12];
        move16();
        mem_fx[12] = mem_temp;
        move16();
    }
    return;
}


/*-------------------------------------------------------------------*
* decimate_3_over_2_allpass_fx()
*
* Decimate 2/3 using allpass iir polyphase filter. Delay 4 samples @48k
*-------------------------------------------------------------------*/

void decimate_2_over_3_allpass_fx(
    const Word16 *input_fx,           /* i  : input signal            */ /* Q_input */
    const Word16 len,                 /* i  : number of input samples */
    Word16 *out_fx,             /* o  : output signal           */ /* Q_input */
    Word16 *mem_fx,             /* i/o: memory                  */ /* Q_input */
    const Word16 *filt_coeff_fx,      /* i  : filter coefficients     */ /* Q15*/
    const Word16 *lp_num_fx,           /* i  : Num Coefficients : Q15 */
    const Word16 *lp_den_fx,           /* o  : Den Coefficients : Q15 */
    Word16 *lp_mem_fx           /* o  : Filter memories  : Q_input */
)
{

    Word16 i, loop_len;
    Word16 Vu1, Vm1, Vl1;    /* Outputs of three cascaded allpass stages (upper, middle, and lower) */
    Word16 Vu0=0, Vm0=0, Vl0=0; /* initialize just to avoid compiler warnings */
    Word16 out1_buff[L_FRAME48k*2];
    Word16 *out1, *in;
    Word16 out, tmp;
    Word16 mem_fx_11, mem_fx_7, mem_fx_3;

    out1 = out1_buff;
    /* *out1++ = lp_num[0] * ( input[0] + lp_mem[0] ) - lp_den[2] * lp_mem[2]; */
    *out1++ = msu_r(L_mult(lp_num_fx[0], add(input_fx[0], lp_mem_fx[0])), lp_den_fx[2], lp_mem_fx[2]);
    move16();
    /* *out1++ = lp_num[1] * input[0] - lp_den[2] * lp_mem[1]; */
    *out1++ = msu_r(L_mult(lp_num_fx[1], input_fx[0]), lp_den_fx[2], lp_mem_fx[1]);
    move16();

    FOR (i=1; i < len; i++)
    {
        /* tmp = lp_num[0] * ( input[i] + input[i-1] ) - lp_den[2] * out1[-2];
           *out1++ = tmp; */
        tmp = msu_r(L_mult(lp_num_fx[0], add(input_fx[i], input_fx[i-1])), lp_den_fx[2], out1[-2]);
        *out1++ = tmp;
        move16();
        /* tmp = lp_num[1] * input[i] - lp_den[2] * out1[-2];
           *out1++ = tmp; */
        tmp = msu_r(L_mult(lp_num_fx[1], input_fx[i]), lp_den_fx[2], out1[-2]);
        *out1++ = tmp;
        move16();
    }
    lp_mem_fx[0] = input_fx[len-1];
    move16();
    lp_mem_fx[1] = out1[-1];
    move16();
    lp_mem_fx[2] = out1[-2];
    move16();

    /* do the all pass polyphase filter with pi/3 cutoff */
    out1 = out_fx;
    in = out1_buff;
    /* loop_len = len*2/3 */
    loop_len = shl(len,1)/3; /* Replace with a better way to divide by 3 when len is divisible by 3 */

    assert(loop_len > 0);

    mem_fx_11 = mem_fx[11];
    move16();
    mem_fx_7 = mem_fx[7];
    move16();
    mem_fx_3 = mem_fx[3];
    move16();
    FOR (i = 0; i < loop_len; i++ )
    {
        /* Lower branch */
        /*Vl0 = mem[8] + filt_coeff[6] * (*in - mem[9]);
          Vl1 = mem[9] + filt_coeff[7] * (Vl0 - mem[10]);
          mem[11] = mem[10] + filt_coeff[8] * (Vl1 - mem[11]); */
        tmp = mult_r(filt_coeff_fx[6], sub(*in++,  mem_fx[9]));
        if (i == 0)
        {
            Vl0 = add(mem_fx[8] , tmp);
        }
        if (i != 0)
        {
            Vl0 = add(*(in-4) , tmp);
        }
        Vl1       = add(mem_fx[9] , mult_r(filt_coeff_fx[7], sub(Vl0, mem_fx[10])));
        mem_fx_11 = add(mem_fx[10], mult_r(filt_coeff_fx[8], sub(Vl1, mem_fx_11)));

        /* mem[8] = *in++;
          mem[9] = Vl0;
          mem[10] = Vl1;
          *out1 = mem[11]; */

        mem_fx[9] = Vl0;
        move16();
        mem_fx[10] = Vl1;
        move16();

        /* Middle branch */
        /* Vm0 = mem[4] + filt_coeff[3] * (*in - mem[5]);
           Vm1 = mem[5] + filt_coeff[4] * (Vm0-mem[6]);
           mem[7] = mem[6] + filt_coeff[5] * (Vm1-mem[7]); */
        tmp = mult_r(filt_coeff_fx[3], sub(*in++, mem_fx[5]));
        if (i == 0)
        {
            Vm0 = add(mem_fx[4] , tmp);
        }
        if (i != 0)
        {
            Vm0 = add(*(in-4), tmp);
        }
        Vm1       = add(mem_fx[5] , mult_r(filt_coeff_fx[4], sub(Vm0, mem_fx[6])));
        mem_fx_7  = add(mem_fx[10], mult_r(filt_coeff_fx[5], sub(Vm1, mem_fx_7)));

        mem_fx[5] = Vm0;
        move16();
        mem_fx[6] = Vm1;
        move16();
        out = add(mem_fx_11, mem_fx_7);

        /* Upper branch */
        /* Vu0 = mem[0] + filt_coeff[0] * ( *in - mem[1] );
           Vu1 = mem[1] + filt_coeff[1] * ( Vu0 - mem[2] );
           mem[3] = mem[2] + filt_coeff[2] * ( Vu1 - mem[3] ); */
        tmp = mult_r(filt_coeff_fx[0], sub(*in++, mem_fx[1]));
        if (i == 0)
        {
            Vu0 = add(mem_fx[0] , tmp);
        }
        if (i != 0)
        {
            Vu0 = add(*(in-4), tmp);
        }
        Vu1       = add(mem_fx[1] , mult_r(filt_coeff_fx[1], sub(Vu0, mem_fx[2])));
        mem_fx_3  = add(mem_fx[2] , mult_r(filt_coeff_fx[2], sub(Vu1, mem_fx_3)));

        mem_fx[1] = Vu0;
        move16();
        mem_fx[2] = Vu1;
        move16();
        *out1++ = add(out, mem_fx_3);
        move16();
    }
    mem_fx[8] = *(in-3);
    move16();
    mem_fx[4] = *(in-2);
    move16();
    mem_fx[0] = *(in-1);
    move16();
    mem_fx[11] = mem_fx_11;
    move16();
    mem_fx[7] = mem_fx_7;
    move16();
    mem_fx[3] = mem_fx_3;
    move16();
    return;
}


/*-------------------------------------------------------------------*
 * retro_interp4_5_fx()
 *
 *
 *-------------------------------------------------------------------*/

void retro_interp4_5_fx(
    const Word16 *syn_fx,
    Word16 *pst_old_syn_fx
)
{
    Word16 *pf5, *pf4;
    Word16 c;

    /* resample st->pst_old_syn in a reverse way to preserve time-alignment */
    pf4 = (Word16*) &pst_old_syn_fx[58];
    pf5 = (Word16*) pst_old_syn_fx;
    FOR (c=0; c<57; c++)
    {
        *pf5++ = pf4[0];
        move16();
        /* *pf5++ = 0.2f * pf4[0] + 0.8f * pf4[1]; */
        *pf5++ = mac_r(L_mult(6554, pf4[0]), 26214, pf4[1]);
        move16();
        /* *pf5++ = 0.4f * pf4[1] + 0.6f * pf4[2]; */
        *pf5++ = mac_r(L_mult(13107, pf4[1]), 19661, pf4[2]);
        move16();
        /* *pf5++ = 0.6f * pf4[2] + 0.4f * pf4[3]; */
        *pf5++ = mac_r(L_mult(19661, pf4[2]), 13107, pf4[3]);
        move16();
        /* *pf5++ = 0.8f * pf4[3] + 0.2f * pf4[4]; */
        *pf5++ = mac_r(L_mult(26214, pf4[3]), 6554, pf4[4]);
        move16();
        pf4+=4;
    }
    *pf5++ = pf4[0];
    move16();
    /* *pf5++ = 0.2f * pf4[0] + 0.8f * pf4[1]; */
    *pf5++ = mac_r(L_mult(6554, pf4[0]), 26214, pf4[1]);
    move16();
    /* *pf5++ = 0.4f * pf4[1] + 0.6f * pf4[2]; */
    *pf5++ = mac_r(L_mult(13107, pf4[1]), 19661, pf4[2]);
    move16();
    /* *pf5++ = 0.6f * pf4[2] + 0.4f * pf4[3]; */
    *pf5++ = mac_r(L_mult(19661, pf4[2]), 13107, pf4[3]);
    move16();
    /* *pf5++ = 0.8f * pf4[3] + 0.2f * syn[0]; */
    *pf5++ = mac_r(L_mult(26214, pf4[3]), 6554, syn_fx[0]);
    move16();
    /* all samples processed: NBPSF_PIT_MAX = 290 = (58*5) */

    return;
}


/*-------------------------------------------------------------------*
 * retro_interp5_4_fx()
 *
 *
 *-------------------------------------------------------------------*/

void retro_interp5_4_fx( Word16 *pst_old_syn_fx )
{
    Word16 *pf5, *pf4;
    Word16 c;

    /* resample st->pst_old_syn in a reverse way to preserve time-alignment */
    pf4 = (Word16*) &pst_old_syn_fx[NBPSF_PIT_MAX-1];
    pf5 = pf4;
    FOR (c=0; c<58; c++)
    {
        /* *pf4-- = 0.75f * pf5[0] + 0.25f * pf5[-1]; */
        *pf4-- = mac_r(L_mult(24576, pf5[0]), 8192, pf5[-1]);
        move16();
        /* *pf4-- = 0.50f * pf5[-1] + 0.50f * pf5[-2]; */
        *pf4-- = mac_r(L_mult(16384, pf5[-1]), 16384, pf5[-2]);
        move16();
        /* *pf4-- = 0.25f * pf5[-2] + 0.75f * pf5[-3]; */
        *pf4-- = mac_r(L_mult(8192, pf5[-2]), 24576, pf5[-3]);
        move16();
        *pf4-- = pf5[-4];
        move16();
        pf5-=5;
    }
    /* all samples processed: NBPSF_PIT_MAX = 290 = (58*5) */

    return;
}

