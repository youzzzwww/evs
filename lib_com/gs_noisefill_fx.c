/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "rom_com_fx.h"
#include "prot_fx.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * gs_noisf()
 *
 * Noise fill-in function
 *-------------------------------------------------------------------*/

static void gs_noisf_fx(
    const Word16 Start_BIN,     /* i  : First bin for noise fill     */
    const Word16 NB_Qbins,      /* i  : Number of bin per band       */
    const Word16 Noise_fac,    /* i  : Noise level              Q15 */
    const Word16 *y_norm,      /* i  : Quantized pulses         Qn  */
    Word16 *exc_diffQ,   /* o  : Quantized pulses with noise added Qn */
    Word16 *seed_tcx,     /* i  : Random generator seed    */
    const Word16 coder_type,     /* i  : coder type               */
    Word16 qNoise_fac
)
{
    Word32 ftmp;
    Word16 i, k;
    Word16 NB_zer;
    Word32 const_1=1;
    Word16 tmp;

    NB_zer = shr(NB_Qbins,1);

    const_1 = L_shl(const_1, add(qNoise_fac, qNoise_fac));
    if( sub(coder_type,INACTIVE) == 0 )
    {
        NB_zer = 2;
        move16();
    }

    /*----------------------------------------------*
     * noise fill-in on unquantized subvector       *
     * injected only from 1066Hz to 6400Hz.         *
     *----------------------------------------------*/

    FOR( k=Start_BIN; k<NB_Qbins + Start_BIN; k+=NB_zer )
    {
        ftmp = L_deposit_l(0);
        FOR(i=k; i<k+NB_zer; i++)
        {
            exc_diffQ[i] = y_norm[i];
            move16();
            ftmp = L_mac0(ftmp, exc_diffQ[i], exc_diffQ[i]);
        }

        IF (L_sub(L_shl(ftmp, 1),const_1) < 0)
        {
            FOR(i=k; i<k+NB_zer; i++)
            {
                /*exc_diffQ[i] += Noise_fac*((float)own_random(seed_tcx)/32768.0f);*/
                tmp = mult(Noise_fac, Random(seed_tcx));/*Q15 */
                tmp = shr(tmp, sub(15,qNoise_fac));/*qNoise_fac */
                exc_diffQ[i] = add(exc_diffQ[i], tmp);
                move16();/*Q */
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * EstimateNoiseLevel_inner()
 *
 * Estimate noise level from the power spectrum
 *-------------------------------------------------------------------*/

static void EstimateNoiseLevel_inner_fx(
    Word16 *noisepb,     /* o  : Noise per band Q15 */
    const long  bitrate,       /* i  : Bitrate of the codec */
    const Word16 i_band,       /* i  : First band to compute the noise  */
    const Word16 Mbands_gn     /* i  : number of bands                  */
)
{
    Word16 i;
    Word16 noise_offset;

    noise_offset = 8192;
    move16();
    /*0.25f * 32768 */
    IF( bitrate > ACELP_24k40 )
    {
        noise_offset = 6554;
        move16(); /*.2f * 32768 */
    }
    ELSE IF ( bitrate >= ACELP_22k60 )
    {
        noise_offset = 9830;
        move16();/*.3f * 32768 */
    }
    ELSE IF ( bitrate >= ACELP_9k60 )
    {
        noise_offset = 11469;
        move16(); /*0.35f * 32768 */
    }
    ELSE
    {
        noise_offset = 13107;
        move16(); /*.4f * 32768 */
    }

    set16_fx( noisepb + i_band, noise_offset, sub(Mbands_gn, i_band) );

    FOR( i = i_band; i < 5; i++ )
    {
        noisepb[i] = s_min(noisepb[i], 6554);
        move16();
    }

    return;
}
/*==========================================================================*/
/* FUNCTION : void  EstimateNoiseLevel_fx()       	    					*/
/*--------------------------------------------------------------------------*/
/* PURPOSE  :                                                       	    */
/*--------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												        */
/* _ (Word32) bitrate              : Bitrate of the codec 	Q0	            */
/* _ (Word16) Diff_len  		   : number of bin before cut-off frequency */
/* _ (Word16) Mbands_gn            : number of bands 		Q0              */
/* _ (Word16) coder_type           : coder type             Q0              */
/* _ (Word16) noise_lev  		   : pulses dynamic         Q0            	*/
/* _ (Word16) pit_band_idx         : bin position of the cut-off frequency  */
/* _ (Word16*) freq_nsbin_per_band : bin per bands tables 	Q0				*/
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													    */
/* _ (Word16*) noisepb	           :  Noise per band        Q15             */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											    */
/*  None                                                                    */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													    */
/* _ None																    */
/*==========================================================================*/
static void EstimateNoiseLevel_fx(
    Word16 *noisepb,             /* o  : Noise per band                          */
    const Word32  bitrate,              /* i  : Bitrate of the codec                    */
    const Word16 Diff_len,             /* i  : number of bin before cut-off frequency  */
    const Word16 Mbands_gn,            /* i  : number of bands                         */
    const Word16 coder_type,           /* i  : coder type                              */
    const Word16 noise_lev,            /* i  : pulses dynamic                          */
    const Word16 pit_band_idx,         /* i  : bin position of the cut-off frequency   */
    Word16 last_bin,             /* i  : the last bin of bit allocation          */
    Word16 bwidth
)
{
    Word16 i_band;

    i_band = 0;
    move16();

    IF( sub(Diff_len,L_FRAME) < 0)
    {
        EstimateNoiseLevel_inner_fx(noisepb, bitrate, i_band, MBANDS_GN);
        IF( coder_type != INACTIVE )
        {
            test();
            test();
            IF( (L_sub(bitrate,ACELP_8k00) == 0 && sub(last_bin,8) > 0) && sub(bwidth,NB) != 0)
            {
                FOR( ; Mbands_gn > i_band;  i_band++)
                {
                    noisepb[i_band] = add(noisepb[i_band],noisepb[i_band]);
                    move16();
                }
            }
            ELSE
            {
                FOR( ; pit_band_idx > i_band; i_band++ )
                {
                    noisepb[i_band] = mult_r(noisepb[i_band], 16384);
                    move16();/* 1/2=0.5 in Q15 */
                }
            }
        }
    }
    test();
    IF ( (sub(coder_type,INACTIVE) == 0 || sub(noise_lev,NOISE_LEVEL_SP3) >= 0) )
    {
        FOR( i_band = 9; i_band < Mbands_gn; i_band++ )
        {
            noisepb[i_band] = add(noisepb[i_band], mult_r(noisepb[i_band], 4915));
            move16();/*noisepb[i_band]*1.15=noisepb[i_band] *(1 + 0.15) */
        }
    }

    return;
}

/*============================================================================*/
/* FUNCTION : void  Appy_NoiseFill_fx()            	    					  */
/*----------------------------------------------------------------------------*/
/* PURPOSE  :                                                       	      */
/*----------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												          */
/* _ (Word16*) seed_tcx           : Seed for noise           Q0               */
/* _ (Word16*) noisepb  		  : Noise per band           Q15              */
/* _ (Word16) Diff_len            : number of bin before cut-off frequency Q0 */
/* _ (Word16) Mbands_gn           : number of bands          Q0               */
/* _ (Word16) coder_type  		  : pulses dynamic           Q0            	  */
/* _ (Word16*) freq_nsbin_per_band: bin per bands tables     Q0               */
/* _ (Word16)  qexc_diffQ         : Q format of exc_diffQ      			      */
/*----------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													      */
/* _ (Word16*) exc_diffQ	      :  Noise per band        qexc_diffQ         */
/*----------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											      */
/*  None                                                                      */
/*----------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													      */
/* _ None																      */
/*============================================================================*/
static void Apply_NoiseFill_fx(
    Word16 *exc_diffQ,             /* i/o: Noise per band                        qexc_diffQ  */
    Word16 *seed_tcx,              /* i  : Seed for noise                                     */
    const Word16 *noisepb,               /* i  : Noise per band                        Q15         */
    const Word16 Diff_len,               /* i  : number of bin before cut-off frequency  */
    const Word16 Mbands_gn,              /* i  : number of bands                         */
    const Word16 coder_type,             /* i  : coder type                              */
    const Word16 *freq_nsbin_per_band,   /* i  : bin per bands tables                    */
    Word16 qexc_diffQ
)
{
    Word16 StartBin, NB_Qbins, i_band;
    StartBin = 0;
    move16();
    NB_Qbins  = 0;
    move16();

    FOR( i_band = 0; i_band < Mbands_gn; i_band++ )
    {
        StartBin += NB_Qbins;
        move16();
        NB_Qbins = freq_nsbin_per_band[i_band];
        move16();

        IF( sub(Diff_len,L_FRAME) < 0 )
        {
            gs_noisf_fx( StartBin , NB_Qbins, noisepb[i_band], exc_diffQ, exc_diffQ, seed_tcx, coder_type,  qexc_diffQ);
        }
    }

    return;
}
/*==========================================================================*/
/* FUNCTION :void freq_dnw_scaling_fx	()							        */
/*--------------------------------------------------------------------------*/
/* PURPOSE  :                                                               */
/*--------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												        */
/* _ (Word16) cor_strong_limit       : HF correlation		  Q0		    */
/* _ (Word16) coder_type 			 : coder type			  Q0		    */
/* _ (Word16) noise_lev				 : Noise level 			  Q0		    */
/* _ (Word32) core_brate             : Core bitrate           Q0            */
/* _ (Word16) Qx                     : Q format of fy_norm				    */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													    */
/* _ None                                                                   */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											    */
/* _ (Word16[]) fy_norm        : Frequency quantized parameter  Qx         	*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													    */
/* _None                                                         		    */
/*==========================================================================*/
void freq_dnw_scaling_fx(
    const Word16 cor_strong_limit, /* i  : HF correlation                */
    const Word16 coder_type,       /* i  : coder type                    */
    const Word16 noise_lev,        /* i  : Noise level                   */
    const Word32  core_brate,       /* i  : Core bitrate                  */
    Word16 fy_norm[],        /* i/o: Frequency quantized parameter */
    Word16 Qx				/* Q format of fy_norm*/
)
{
    Word16 sc_dyn;
    Word16 start_sc, i;

    sc_dyn = 32767;
    move16(); /*Q15 */
    start_sc = L_FRAME;
    move16();
    test();
    IF( L_sub(core_brate,ACELP_8k00) <= 0 && sub(coder_type,INACTIVE) == 0 )
    {
        sc_dyn = mult_r(sc_dyn,4915);  /*Q15  (0.15 in Q15) */
        start_sc = 64;
        move16();
    }
    ELSE IF ( sub(coder_type,INACTIVE) == 0 )
    {
        sc_dyn = mult_r(sc_dyn,8192);   /*Q15 (0.25 in Q15) */
        start_sc = 80;
        move16();
    }
    ELSE
    {
        /*sc_dyn = (float)(NOISE_LEVEL_SP3 - noise_lev)/10.0f + 0.4f;*/
        sc_dyn =  extract_l(L_mac(13107, sub(NOISE_LEVEL_SP3, noise_lev), 1638));   /*Q0*Q14x2+Q15 =Q15*/
        start_sc = add(112, shl(sub(NOISE_LEVEL_SP3, noise_lev), 4));
        if( sub(noise_lev,NOISE_LEVEL_SP0) == 0)
        {
            start_sc = L_FRAME;
            move16();
        }
    }

    FOR(i = start_sc; i < L_FRAME; i++)
    {
        fy_norm[i] = mult_r(fy_norm[i],sc_dyn);
        move16();/*Qx */
    }

    test();
    test();
    IF( (L_sub(core_brate,ACELP_13k20) < 0 && cor_strong_limit  == 0) || L_sub(core_brate,ACELP_9k60) < 0)
    {
        FOR(i = 160; i < L_FRAME; i++)
        {
            fy_norm[i] = s_min(fy_norm[i],shl(1,Qx));
            move16();
            fy_norm[i] = s_max(fy_norm[i],shl(-1,Qx));
            move16();
        }
    }
    ELSE IF ( L_sub(core_brate,ACELP_22k60) < 0 )
    {
        FOR(i = 160; i < L_FRAME; i++)
        {
            fy_norm[i] = s_min(fy_norm[i],shr_r(1536,sub(10,Qx)));
            move16();
            fy_norm[i] = s_max(fy_norm[i],shr_r(-1536,sub(10,Qx)));
            move16();
        }
    }

    return;

}

static void Decreas_freqPeak_fx(
    Word16 *lsf_new,              /* i  : ISFs at the end of the frame                          */
    Word16 *exc_diffQ,            /* i/o: frequency coefficients of per band                    */
    Word16 rat                    /* i  : threshold of ratio between consecutive lsf_new_diff   */
)
{
    Word16 i, j, k;
    Word16 last_bin = 0;
    Word16 pos = 0;
    Word16 *src, max,avrg;
    Word32 L_avrg,L_tmp;
    Word16 lsf_new_diff[M];
    Word16 tmp,tmp1,exp;

    move16();   /*ptr init*/
    FOR(j=1; j<(M-1); j++)
    {
        lsf_new_diff[j] =sub( lsf_new[j] , lsf_new[j-1]);/*Qx2.56 */
    }

    avrg = 0;
    move16();
    L_avrg = L_deposit_l(0);
    max = 1;
    move16();
    FOR(i=160; i<L_FRAME; i++)
    {
        IF(sub(abs_s(exc_diffQ[i]),max) > 0)
        {
            max = abs_s(exc_diffQ[i]);
            pos = i;
            move16();
        }
        L_avrg = L_add(L_avrg,abs_s(exc_diffQ[i]));
    }
    /* avrg /= 96; */
    L_avrg = Mult_32_16(L_avrg,21845);/*Q_exc+21 -15 ->Q_exc + 6 */
    avrg = round_fx(L_shl(L_avrg,10));/*Q_exc */

    FOR(i=0; i<(M-1); i++)
    {
        if(sub(lsf_new[i],10240) > 0)
        {
            last_bin = i;
            move16();
            BREAK;
        }
    }

    FOR(i=last_bin; i<14; i++)
    {
        tmp = mult_r(rat,lsf_new_diff[i-1] );/*Qx2.56 */
        IF(sub(tmp , lsf_new_diff[i])>0)
        {
            src = &exc_diffQ[shl(sub(i,1),4)];
            move16();
            FOR(j=0; j<2; j++)
            {
                FOR(k=0; k<16; k++)
                {
                    tmp = mult_r(16384,abs_s(*src));
                    IF(sub(tmp,avrg)>0)
                    {
                        tmp = abs_s(*src) ;
                        exp = norm_s(max);
                        tmp1 = div_s(1<<(14- exp),max);/*Q(29 - exp - Q_exc) */
                        L_tmp = L_mult(tmp,tmp1);/*Q(30 - exp) */
                        tmp = round_fx(L_shl(L_tmp,exp));/*Q14 */
                        tmp = sub(32767,tmp);/*Q14 */
                        L_tmp = L_mult(avrg,tmp);/*Q_exc +15 */

                        tmp  =  round_fx(L_shl(L_tmp,1));
                        tmp1 = negate(tmp);

                        *(src) = (*src > 0) ? tmp :tmp1 ;
                        move16();
                        logic16();

                        /* *(src) = (*src > 0) ? (float)(avrg*(2.0f-fabs(*src)/max)) : (float)(-avrg*(2.0f-fabs(*src)/max));  */
                    }
                    src++;
                }
            }
        }
    }

    tmp = mult_r(8192,max);/*Q_exc */
    test();
    IF(sub(abs_s(exc_diffQ[pos]),max) == 0 && sub(tmp ,avrg)>0)
    {
        FOR(i=pos-1; i<pos+2; i++)
        {
            exc_diffQ[pos] =mult_r(16384,exc_diffQ[pos]);
            move16();
        }
    }

    return;
}

static void envelop_modify_fx(
    Word16 *exc_diffQ_fx,             /* i/o: frequency coefficients of per band      */
    Word16 *seed_tcx,                 /* i  : Seed for noise                          */
    Word16 last_bin,                  /* i  : last bin of bit allocation              */
    Word16 *Ener_per_bd_iQ_fx,        /* i  : Quantized energy of targeted vector     */
    Word16 Q_exc,
    Word16 *Q_hb_exc
)
{
    Word16 i, j, end_band;
    Word16 start_band;
    Word32 Ener_fx;
    Word16 Ener1_fx;
    Word16 tmp, tmp1;
    Word32 L_tmp;
    Word16 exp, exp1, frac;
    Word16 *src_fx;
    Word16 weight_fx;
    Word32 L_exc_diffQ_fx[L_FRAME16k], exc_diffQ_max;
    Word16 Q_tmp;

    start_band = i_mult(last_bin, 16);
    end_band = L_FRAME;
    move16();
    Ener_fx = L_deposit_l(0);
    FOR(i=start_band; i<end_band; i++)
    {
        L_tmp = L_mult0(exc_diffQ_fx[i], exc_diffQ_fx[i]); /*2*Q_exc */
        Ener_fx = L_add(Ener_fx, L_shr(L_tmp, 7)); /*2*Q_exc-7 */
    }

    tmp = sub(end_band, start_band);
    tmp = div_s(1, tmp);/*Q15 */
    Ener_fx = Mult_32_16(Ener_fx, tmp); /*Q(2*Q_exc-7+15)->Q(2*Q_exc-7) */

    exp1 = norm_l(Ener_fx);
    Ener_fx = L_shl(Ener_fx, exp1);
    exp1 = 31-exp1-sub(shl(Q_exc,1),7);
    move16();
    Ener_fx = Isqrt_lc(Ener_fx, &exp1); /*Q(31-exp1) */

    weight_fx = 16384; /*Q15 */
    src_fx = &exc_diffQ_fx[start_band]; /*Q_exc */
    FOR(i=last_bin; i<last_bin+4; i++)
    {
        /*Ener1 = (float)(0.4f*pow(10, Ener_per_bd_iQ[i+1])); */
        L_tmp = L_shr(L_mult0(Ener_per_bd_iQ_fx[i+1], 27213), 9); /* 3.321928 in Q13 -> Q16 */

        frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */
        tmp = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp = sub(exp, 14);
        Ener1_fx = mult_r(13107, shl(tmp, exp));  /*Q0 */

        FOR(j=0; j<16; j++)
        {
            /**src = Ener1*(weight*(*src)*Ener + (1.0f-weight)*own_random(seed_tcx)/32768.0f); */
            L_tmp = Mult_32_16(Ener_fx, *src_fx); /*Q(31-exp+Q_exc-15) -> Q(16-exp+Q_exc) */
            tmp = extract_l(L_shr(L_tmp, add(4, sub(Q_exc, exp1)))); /*Q12 */
            tmp = mult_r(weight_fx, tmp); /*Q12 */

            L_tmp = L_mult0(sub(32767, weight_fx), Random(seed_tcx)); /*Q30 */
            tmp1 = round_fx(L_shr(L_tmp, 2));

            L_exc_diffQ_fx[16*i+j] = L_mult0(Ener1_fx, add(tmp, tmp1));     /*Q12           */  move32();
            src_fx++;
        }
    }

    /*Ener1 = (float)(0.4f*pow(10, Ener_per_bd_iQ[15])); */
    L_tmp = L_shr(L_mult0(Ener_per_bd_iQ_fx[15], 27213), 9); /* 3.321928 in Q13 -> Q16 */

    frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */
    tmp = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
    /* output of Pow2() will be: */
    /* 16384 < Pow2() <= 32767 */
    exp = sub(exp, 14);
    Ener1_fx = mult_r(13107, shl(tmp, exp));  /*Q0 */

    src_fx = &exc_diffQ_fx[224];
    FOR(j=0; j<32; j++)
    {
        /**src = Ener1*(weight*(*src)*Ener + (1.0f-weight)*own_random(seed_tcx)/32768.0f); */
        L_tmp = Mult_32_16(Ener_fx, *src_fx); /*Q(31-exp+Q_exc-15) -> Q(16-exp+Q_exc) */
        tmp = extract_l(L_shr(L_tmp, add(4, sub(Q_exc, exp1)))); /*Q12 */
        tmp = mult_r(weight_fx, tmp); /*Q12 */

        L_tmp = L_mult0(sub(32767, weight_fx), Random(seed_tcx)); /*Q30 */
        tmp1 = round_fx(L_shr(L_tmp, 2));  /*Q12 */

        L_exc_diffQ_fx[16*i+j] = L_mult0(Ener1_fx, add(tmp, tmp1));     /*Q12                     */  move32();
        src_fx++;
    }

    exc_diffQ_max = 0;
    move16();
    FOR(i=start_band; i<L_FRAME; i++)
    {
        IF(L_sub(L_abs(L_exc_diffQ_fx[i]), exc_diffQ_max) > 0)
        {
            exc_diffQ_max = L_abs(L_exc_diffQ_fx[i]);
        }
    }
    exp = norm_l(exc_diffQ_max);

    IF(sub(exp,16) > 0)
    {
        *Q_hb_exc = 12;
        move16();
        FOR(i=start_band; i<L_FRAME; i++)
        {
            exc_diffQ_fx[i] = extract_l(L_exc_diffQ_fx[i]);
        }
    }
    ELSE
    {
        Q_tmp = sub(16, exp);
        *Q_hb_exc = sub(12, Q_tmp);
        FOR(i=start_band; i<L_FRAME; i++)
        {
            exc_diffQ_fx[i] = extract_l(L_shr(L_exc_diffQ_fx[i], Q_tmp));
        }
    }

    return;
}

void highband_exc_dct_in_fx(
    const Word32 core_brate,                 /* i  : core bitrate                            */
    const Word16 *mfreq_bindiv_loc,         /* i  : bin per bands tables                    */
    Word16 last_bin,                  /* i  : last bin of bit allocation              */
    Word16 Diff_len,                  /* i  : number of bin before cut-off frequency  */
    Word16 noise_lev,                 /* i  : pulses dynamic                          */
    Word16 pit_band_idx,              /* i  : bin position of the cut-off frequency   */
    Word16 *exc_diffQ,                /* i  : frequency coefficients of per band      */
    Word16 *seed_tcx,                 /* i  : Seed for noise                          */
    Word16 *Ener_per_bd_iQ,           /* i  : Quantized energy of targeted vector     */
    Word16 nb_subfr,                  /* i  : Number of subframe considered           */
    Word16 *exc_dct_in,               /* o  : dct of residual signal                  */
    Word16 last_coder_type,           /* i  : coding type of last frame               */
    Word16 *bitallocation_band,       /* i  : bit allocation flag of each band        */
    Word16 *lsf_new,                  /* i  : LSFs at the end of the frame            */
    Word16 *last_exc_dct_in,          /* i  : dct of residual signal of last frame    */
    Word16 *last_ener,                /* i  : frequency energy  of last frame         */
    Word16 *last_bitallocation_band,  /* i  : bit allocation flag of each band  of last frame   */
    Word16 *bitallocation_exc,        /* i  : flag of decoded coefficients            */
    Word16 bfi,                       /* i  : bad frame indicator                     */
    const Word16 coder_type,                /* i  : coder type                              */
    Word16 bwidth,
    Word16 *exc_wo_nf ,                /* o  : temporal excitation (in f domain) without noisefill   */
    Word16 Qexc_diffQ,
    Word16 Q_exc,
    const Word16 GSC_noisy_speech
)
{
    Word16 i, j, k;
    Word16 MAX_Bin = 0;
    Word16 last_bin_tmp,ener=0;
    Word16 noisepb[MBANDS_GN];
    Word16 Ener_per_bd_yQ[MBANDS_GN];
    Word16 *src, *dst;
    Word32 L_tmp;
    Word16 length_bin, bwe_flag = 0,tmp;
    Word16 frac,exp,tmp1;
    Word16 *end, Q_hb_exc;

    FOR( j=10; j<MBANDS_GN; j++ )
    {
        /*  ener += (float)pow(10, Ener_per_bd_iQ[j]);
            ener += (float)pow(2, 3.321928*Ener_per_bd_iQ[j]); */

        L_tmp = L_mult(Ener_per_bd_iQ[j], 27213); /* 3.321928 in Q13 -> Q27 */
        L_tmp = L_shr(L_tmp, 10); /* From Q27 to Q16 */

        frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of L_tmp */
        tmp = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
        /* output of Pow2() will be: */
        /* 16384 < Pow2() <= 32767 */
        exp = sub(exp, 14);
        tmp1 = shl(tmp,add(exp,0));
        ener = add (tmp1,ener);/*Q0 */
    }

    test();
    IF( L_sub(core_brate,ACELP_8k00) == 0 && sub(bwidth,NB) != 0 )
    {
        if(sub(last_coder_type,AUDIO) != 0)
        {
            *last_ener = ener;
            move16();
        }
        test();
        test();
        IF((sub(last_bin,8) > 0 || Diff_len != 0) && sub(last_coder_type,AUDIO) == 0)
        {
            MAX_Bin = 10;
            move16();
            bwe_flag = 1;
            move16();
        }
        ELSE
        {
            MAX_Bin = 15;
            move16();
        }

        last_bin_tmp = last_bin;
        move16();
        last_bin  = s_max(last_bin , MAX_Bin);
        last_bin = add(last_bin, 1);
    }
    ELSE
    {
        last_bin = MBANDS_GN;
        move16();
        last_bin_tmp = last_bin;
        move16();
    }
    IF( bfi )
    {
        set16_fx( noisepb, 13107, MBANDS_GN ); /*0.4 in Q15 */
    }
    ELSE
    {
        EstimateNoiseLevel_fx( noisepb, core_brate, Diff_len, last_bin, coder_type, noise_lev, pit_band_idx,
        last_bin_tmp, bwidth );
    }

    IF( exc_wo_nf != NULL )
    {
        Copy( exc_diffQ, exc_wo_nf, L_FRAME );
    }

    test();
    IF( GSC_noisy_speech && !bfi )
    {
        set16_fx( noisepb, 3277, MBANDS_GN );
    }
    Apply_NoiseFill_fx( exc_diffQ, seed_tcx, noisepb, Diff_len, last_bin, coder_type, mfreq_bindiv_loc,Qexc_diffQ );

    /*--------------------------------------------------------------------------------------*
     * Quantize average gain
     * Substract Q averaged gain
     * VQ of remaining gain per band
     *--------------------------------------------------------------------------------------*/
    test();
    IF( L_sub(core_brate,ACELP_8k00) == 0 && sub(bwidth,NB) != 0 )
    {
        Ener_per_band_comp_fx(exc_diffQ, Ener_per_bd_yQ, Qexc_diffQ, last_bin+1, 0);
    }
    ELSE
    {
        Ener_per_band_comp_fx(exc_diffQ, Ener_per_bd_yQ, Qexc_diffQ, MBANDS_GN, 1 );

        IF( sub(nb_subfr, 4) < 0 )
        {
            FOR(i = L_FRAME-16; i < L_FRAME; i++)
            {
                /*exc_diffQ[i] *= 0.067f * i - 15.0f; = -15 - (-0.067f * i) */
                tmp = msu_r(-7680<<16, -17564, shl(i,6));/*-15 in Q9; -0.067 in Q18 and i in Q6= Q9 */
                L_tmp = L_mult(exc_diffQ[i],tmp); /*Q(Qexc_diffQ+10) */
                exc_diffQ[i] = round_fx(L_shl(L_tmp,16-10));/*Qexc_diffQ */
            }
        }
    }
    /*--------------------------------------------------------------------------------------*
     * Apply decoded gain onto the difference signal
     *--------------------------------------------------------------------------------------*/
    IF( GSC_noisy_speech )
    {
        FOR( i= 0; i < L_FRAME; i++ )
        {
            exc_diffQ[i] = mult_r(exc_diffQ[i], 29491);
            move16();
        }
    }

    Comp_and_apply_gain_fx( exc_diffQ, Ener_per_bd_iQ, Ener_per_bd_yQ, last_bin, 0, Qexc_diffQ, Q_exc );

    IF( exc_wo_nf != NULL )
    {
        Comp_and_apply_gain_fx( exc_wo_nf, Ener_per_bd_iQ, Ener_per_bd_yQ, last_bin, 1 , Qexc_diffQ, Q_exc);
        Vr_add( exc_dct_in, exc_wo_nf, exc_wo_nf, L_FRAME );
    }
    /*--------------------------------------------------------------------------------------*
     * add the correction layer to the LF bins,
     * and add the quantized pulses or the noise for the higher part of the spectrum
     * (non valuable temporal content already zeroed)
     * DC is Zeroed
     *--------------------------------------------------------------------------------------*/

    Vr_add( exc_dct_in, exc_diffQ, exc_dct_in, L_FRAME );
    test();
    IF( core_brate == ACELP_8k00 && bwidth != NB )
    {
        IF( sub(bwe_flag,1) == 0 )
        {
            last_bin = sub(last_bin, 1);
            tmp = i_mult(MAX_Bin, 16);
            tmp1 = i_mult(last_bin, 16);
            src = &exc_diffQ[sub(L_FRAME,1)];
            move16();
            dst = &exc_dct_in[sub(tmp,1)];
            move16();
            end = &exc_diffQ[sub(tmp1,1)];
            move16();

            WHILE (src> end)
            {
                *src-- = *dst--;
                move16();
            }
            test();
            test();
            if( (bitallocation_exc[0] != 0 || bitallocation_exc[1] != 0) && L_sub(core_brate, ACELP_8k00) == 0 )
            {
                exc_diffQ[160] = 0;
                move16();
            }

            Q_hb_exc = 0;
            move16();
            envelop_modify_fx( exc_diffQ, seed_tcx, last_bin, Ener_per_bd_iQ, Q_exc, &Q_hb_exc);
            Copy_Scale_sig( &exc_diffQ[tmp1], &exc_dct_in[tmp1], sub(L_FRAME,tmp1), sub(Q_exc, Q_hb_exc)); /* from Q_hb_exc -> Q_exc as expected */
        }

        IF( sub(nb_subfr,4) < 0 )
        {
            FOR( i = sub(L_FRAME,16); i < L_FRAME; i++ )
            {
                /*exc_dct_in[i] *= (0.067f*i-15.f); */
                tmp = mult_r(17564,shl(i,6));    /*0.067 in Q18 and i in Q6= Q9 */
                tmp = sub(tmp,7680);             /*15 in Q9 = Q9 */
                L_tmp = L_mult(exc_dct_in[i],tmp);/*Q(Q_exc+10) */
                exc_dct_in[i] = round_fx(L_shl(L_tmp,6));/*Q_exc */
            }
        }

        tmp1 = mult_r(ener,16384);
        tmp1 = sub(*last_ener,tmp1);
        tmp =  mult_r(*last_ener,16384);
        tmp =  sub(ener,tmp);
        test();
        IF( tmp>0 && tmp1>0 )
        {
            length_bin = 6;
            move16();
            IF(last_coder_type != AUDIO)
            {
                set16_fx( last_bitallocation_band, 0, MBANDS_GN );
                Copy( &exc_dct_in[(4+length_bin)*16], &last_exc_dct_in[(4+length_bin)*16], length_bin*16 );
            }

            FOR(i=4; i<(4+length_bin); i++)
            {
                test();
                IF( !(bitallocation_band[i] == 0 && last_bitallocation_band[i] == 0))
                {
                    k = shl(add(i,length_bin),4);
                    src = &exc_dct_in[k];     /*(i+length_bin)*16*/
                    dst = &last_exc_dct_in[k];
                    FOR(j=0; j<16; j++)
                    {
                        tmp= mult_r(10923,abs_s(*src));
                        tmp1 =mult_r(10923,abs_s(*dst));

                        IF(sub(tmp,abs_s(*dst)) >0)
                        {
                            test();
                            *src = (*src > 0) ? mult_r(16384,add(*src , abs_s(*dst))) : mult_r(16384,sub(*src , abs_s(*dst))); /*Q_exc */
                        }
                        ELSE IF (sub(tmp1,abs_s(*src)) >0)
                        {
                            tmp = mult_r(*src,22938);
                            tmp1 = mult_r(9830,abs_s(*dst));
                            /*   *src = (*src > 0) ? (0.7f*(*src) + 0.3f*fabs(*dst)) : (0.7f*(*src) - 0.3f*fabs(*dst)); */
                            *src = (*src > 0) ? add(tmp,tmp1) : sub(tmp,tmp1); /*Q_exc */
                        }
                        src++;
                        dst++;
                    }
                }
            }
        }
        IF(sub(bwe_flag,1) == 0)
        {
            Decreas_freqPeak_fx( lsf_new, exc_dct_in, 9830 );
        }
        ELSE
        {
            Decreas_freqPeak_fx( lsf_new, exc_dct_in, 16384 );
        }
    }

    Copy( &exc_dct_in[64], &last_exc_dct_in[64], L_FRAME-64 );
    Copy(bitallocation_band, last_bitallocation_band, 10);
    *last_ener = ener;
    move16();

    return;
}
