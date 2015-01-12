/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "rom_com_fx.h"

/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/
#define A2 6554
#define OmA2 (32768-A2)
#define GAIN_VAR 11811 /* in Q31 divided by 2 (Q30) */

/*-------------------------------------------------------*
 * CNG_exc()
 *
 * Comfort noise generation routine
 *-------------------------------------------------------*/

void CNG_exc_fx(
    const Word32  core_brate,       /* i  : core bitrate                            */
    const Word16 L_frame,          /* i  : length of the frame                      */
    Word32 *Enew,            /* i/o: decoded SID energy                   Q6  */
    Word16 *seed,            /* i/o: random generator seed                    */
    Word16 exc[],            /* o  : current non-enhanced excitation   Q_new  */
    Word16 exc2[],           /* o  : current enhanced excitation       Q_new  */
    Word32 *lp_ener,         /* i/o: LP filtered E                            */
    const Word32 last_core_brate,  /* i  : previous frame core bitrate              */
    Word16 *first_CNG,       /* i/o: first CNG frame flag for energy init.    */
    Word16 *cng_ener_seed,   /* i/o: random generator seed for CNG energy     */
    Word16 bwe_exc[],        /* o  : excitation for SWB TBE                   */
    const Word16 allow_cn_step,    /* i  : allow CN step                            */
    Word16 *last_allow_cn_step,  /* i/o: last allow step                      */
    const Word16 OldQ_exc,          /* i  : Old excitation scaling                  */
    const Word16 Q_exc              /* i  : excitation scaling                      */
    , const Word16 num_ho            /* i  : number of selected hangover frames    */
    ,Word32 q_env[]
    ,Word32 *lp_env
    ,Word32 *old_env
    ,Word16 *exc_mem
    ,Word16 *exc_mem1
    ,Word16 *sid_bw
    ,Word16 *cng_ener_seed1
    ,Word16 exc3[]
    ,Word16 Opt_AMR_WB
)
{
    Word16 i, tmp, tmp2, exp, exp2, Q_ener;
    Word32 L_tmp_ener, L_tmp;
    Word16 i_subfr;
    Word16 pit_max;
    Word16 ftmp,j;
    Word16 *ptR,*ptI;
    Word16 fft_io[L_FRAME16k];
    Word32 itmp[129];
    Word32 env[NUM_ENV_CNG];
    Word32 enr1;
    Word32 denv[NUM_ENV_CNG];
    Word16 fra;
    Word16 temp_lo_fx, temp_hi_fx;
    Word16 exp_pow;
    Word32 L_tmp2;
    Word16 *pt_fft_io;

    /*------------------------------------------------------------------*
     * Initializations
     *------------------------------------------------------------------*/

    pit_max = PIT16k_MAX;
    move16();
    if( sub(L_frame,L_FRAME) == 0 )
    {
        pit_max = PIT_MAX;
        move16();
    }

    /*---------------------------------------------------------------------*
     * Initialization of CNG energy for the first CNG frame
     *---------------------------------------------------------------------*/

    IF(*first_CNG == 0 )
    {
        IF(L_sub(core_brate,FRAME_NO_DATA) == 0 )
        {
            /* needed only in decoder when the very first SID frame was erased and this frame is FRAME_NO_DATA frame */
            /*fenew = dotp( fexc, fexc, pit_max )/pit_max;*/
            L_tmp_ener = Calc_Energy_Autoscaled(exc-pit_max, OldQ_exc, pit_max, &Q_ener);
            L_tmp_ener = Mult_32_16(L_tmp_ener, 9079);   /* divide by PIT_MAX (in Q15 + Q6 to get output in Q6)*/
            L_tmp_ener = L_shr(L_tmp_ener, Q_ener); /* -> If we want ener in Q6 */

            if(sub(L_frame, L_FRAME16k) == 0)
            {
                L_tmp_ener = Mult_32_16(L_tmp_ener, 26214);   /* Compensate for 16kHz */
            }
            *Enew = L_tmp_ener;
            move32();
        }

        *lp_ener = *Enew;
        move32();
    }

    /*---------------------------------------------------------------------*
     * Update CNG energy
     *---------------------------------------------------------------------*/
    test();
    test();
    IF( L_sub(last_core_brate,SID_1k75) != 0 && L_sub(last_core_brate,FRAME_NO_DATA) != 0 && L_sub(last_core_brate,SID_2k40) != 0 )
    {
        /* Partially reset CNG energy after active speech period */
        test();
        IF ( allow_cn_step == 0 && *last_allow_cn_step == 0 )
        {
            test();
            IF( sub(num_ho,3) < 0 || L_sub(Mult_32_16(*Enew,21845 /*1/1.5f, Q15*/), *lp_ener) < 0 )
            {
                /**lp_ener = 0.8f * *lp_ener + 0.2f * *Enew;*/
                L_tmp_ener = Mult_32_16(*lp_ener, 26214);
                L_tmp_ener = Madd_32_16(L_tmp_ener, *Enew, 6554);

            }
            ELSE
            {
                /**lp_ener = 0.95f * *lp_ener + 0.05f * *Enew;*/
                L_tmp_ener = Mult_32_16(*lp_ener, 31130);
                L_tmp_ener = Madd_32_16(L_tmp_ener, *Enew, 1638);
            }
        }
        ELSE
        {
            L_tmp_ener = L_add(0,*Enew);
            *last_allow_cn_step = 0;
            move16();
        }
    }
    ELSE
    {
        /* normal CNG update */
        IF ( *last_allow_cn_step == 0 )
        {
            /**lp_ener = (float)(A2 * *Enew + (1-A2) * *lp_ener);*/
            L_tmp_ener = Mult_32_16(*Enew, A2);
            L_tmp_ener = Madd_32_16(L_tmp_ener, *lp_ener, OmA2);
        }
        ELSE
        {
            test();
            if ( L_sub(core_brate,SID_1k75) == 0 || L_sub(core_brate,SID_2k40) == 0 )
            {
                *last_allow_cn_step = 0;
                move16();
            }

            L_tmp_ener = *Enew;
            move32();

        }
    }
    *lp_ener = L_max(L_tmp_ener,1);
    move32(); /*To avoid / per 0*/

    if ( sub(allow_cn_step,1) == 0)
    {
        *last_allow_cn_step = 1;
        move16();
    }

    /*---------------------------------------------------------------------*
     * Generate white noise vector
     *---------------------------------------------------------------------*/

    /*for ( i=0; i<L_frame; i++ )exc2[i] = (float)own_random( seed );*/
    Random_Fill(seed, L_frame, exc2, 4);
    /*------------------------------------------------------------*
     * Insert random variation for excitation energy
     *  (random variation is scaled according to *lp_ener value)
     *------------------------------------------------------------*/

    FOR ( i_subfr=0; i_subfr<L_frame; i_subfr += L_SUBFR )
    {
        /* ener_lp = own_random(cng_ener_seed) * *lp_ener * GAIN_VAR + *lp_ener */
        /*------------------------------------------------------------*
         * Insert random variation for excitation energy
         *  (random variation is scaled according to *lp_ener value)
         *------------------------------------------------------------*/
        L_tmp = Mult_32_16(*lp_ener, Random(cng_ener_seed));
        L_tmp = Mult_32_16(L_tmp, GAIN_VAR);
        L_tmp = L_add(L_tmp, *lp_ener);
        L_tmp = L_max(L_tmp, 1);

        /* enr = dot_product( exc2, exc2, L_SUBFR ) + 0.01f */
        tmp = extract_h(Dot_product12(&exc2[i_subfr], &exc2[i_subfr], L_SUBFR, &exp));
        exp = add(exp, 8-6);     /* 8 from Q-4, -6 from L_SUBFR */

        /* enr = (float)sqrt(*lp_ener * L_SUBFR / enr) */
        exp2 = norm_l(L_tmp);
        tmp2 = extract_h(L_shl(L_tmp, exp2));
        exp2 = sub(31-6, exp2);  /* in Q15 (L_tmp in Q6)*/

        exp = sub(exp, exp2);

        if (sub(tmp, tmp2) > 0)
        {
            exp = add(exp, 1);
        }
        if (sub(tmp, tmp2) > 0)
        {
            tmp = shr(tmp, 1);
        }
        tmp = div_s(tmp, tmp2);

        L_tmp = L_deposit_h(tmp);

        L_tmp = Isqrt_lc(L_tmp, &exp);
        tmp = extract_h(L_tmp);

        exp = add(exp, 4);       /* From Q15 to Q19 */
        exp = add(exp, Q_exc);   /* Q_exc+ Q19      */

        FOR (i=0; i<L_SUBFR; i++)
        {
            /* exc2[i] *= enr */
            L_tmp = L_mult(exc2[i_subfr+i], tmp); /* Q-4 * Q_exc+19 -> Q_exc +16 */
            exc2[i_subfr+i] = round_fx(L_shl(L_tmp, exp));
        }
    }
    IF ( sub(Opt_AMR_WB,1) != 0 )
    {
        Copy( exc2, exc3, L_FRAME16k);

        /* enr1 = (float)log10( *Enew*L_frame + 0.1f ) / (float)log10( 2.0f ); */
        exp = norm_l(*Enew);
        L_tmp = L_shl(*Enew,exp); /* Q(exp+6) */
        L_tmp = Mult_32_16(L_tmp,shl(L_frame,5)); /* Q(exp+6+5-15=exp-4) */
        L_tmp = L_shr(L_tmp,sub(exp,10)); /* Q6 */

        exp = norm_l(L_tmp);
        fra = Log2_norm_lc(L_shl(L_tmp,exp));
        exp = sub(sub(30,exp),6);
        L_tmp = L_Comp(exp,fra);
        /* enr1 = round_fx(L_shl(L_tmp,8)); */ /*Q8 */
        enr1 = L_shr(L_tmp,10);/* Q6 */


        IF ( L_sub(core_brate,SID_2k40) == 0 )
        {
            IF ( *sid_bw == 0 )
            {
                FOR ( i=0; i<NUM_ENV_CNG; i++ )
                {
                    /* get quantized envelope */
                    /* env[i] = pow(2.0f,(enr1 - q_env[i])); */
                    L_tmp  = L_sub(enr1,q_env[i]);/* Q6 */
                    L_tmp = L_shl(L_tmp, 10);/* 16 */
                    temp_lo_fx = L_Extract_lc(L_tmp, &temp_hi_fx);

                    exp_pow = sub(14, temp_hi_fx);
                    L_tmp = Pow2(14, temp_lo_fx);      /* Qexp_pow */
                    env[i] = L_shl(L_tmp, sub(6, exp_pow));
                    move32();/* Q6 */
                }
            }

            /* initialize CNG envelope */
            test();
            IF( *first_CNG == 0 && *sid_bw == 0 )
            {
                Copy32(env, lp_env, NUM_ENV_CNG);
            }

            IF ( *sid_bw == 0 )
            {
                Copy32(env, old_env, NUM_ENV_CNG);
            }
        }

        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            /* get AR low-passed envelope */
            /* lp_env[i] = 0.9f*lp_env[i] + (1-0.9f)*old_env[i]; */
            L_tmp = Mult_32_16(lp_env[i],29491);
            lp_env[i] = L_add(L_tmp,Mult_32_16(old_env[i],3277));
            move32();/* Q6 */
        }

        /* calculate the spectrum of random excitation signal */
        Copy(exc2, fft_io, L_frame);

        IF ( sub(L_frame,L_FRAME16k) == 0 )
        {
            modify_Fs_fx( fft_io, L_FRAME16k, 16000, fft_io, 12800, exc_mem1, 0 );
        }

        /* fft_rel(fft_io, L_FFT, LOG2_L_FFT); */
        fft_rel_fx(fft_io, L_FFT, LOG2_L_FFT);/* ??????? */
        ptR = &fft_io[1];
        ptI = &fft_io[sub(L_FFT,1)];
        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            /* env[i] = 2.0f*(*ptR * *ptR + *ptI * *ptI)/L_FFT; */
            L_tmp = L_mult(*ptR,*ptR);/* 2*Q_exc+1 */
            L_tmp = L_add(L_tmp,L_mult(*ptI,*ptI));/* 2*Q_exc+1 */
            L_tmp = L_add(L_tmp,L_tmp);/* 2*Q_exc+1 */
            L_tmp = Mult_32_16(L_tmp,128);/* 2*Q_exc+1 */
            tmp = add(add(Q_exc,Q_exc),1);
            env[i] = L_shr(L_tmp,sub(tmp,6));
            move32();/* Q6 */
            ptR++;
            ptI--;
        }

        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            /* denv[i] = lp_env[i] + 2*(*lp_ener) - env[i]; */
            L_tmp = L_add(*lp_ener,*lp_ener);
            denv[i] = L_sub(L_add(lp_env[i],L_tmp),env[i]);
            move32();/* Q6 */

            if ( denv[i] < 0 )
            {
                denv[i] = L_deposit_l(0);
            }
        }

        Copy32(env, itmp, NUM_ENV_CNG);
        set32_fx(itmp, 0, NUM_ENV_CNG);

        set16_fx(fft_io, 0, L_FFT);
        ptR = &fft_io[1];
        ptI = &fft_io[sub(L_FFT,1)];
        FOR (i=0; i<NUM_ENV_CNG; i++)
        {
            /* *ptR = own_random( cng_ener_seed1 ); */
            /* *ptI = own_random( cng_ener_seed1 ); */
            *ptR = Random( cng_ener_seed1 );
            *ptI = Random( cng_ener_seed1 );

            /* env[i] = 2.0f*(*ptR * *ptR + *ptI * *ptI)/L_FFT; */
            L_tmp = L_mult(*ptR,*ptR);/* Q1 */
            L_tmp = L_add(L_tmp,L_mult(*ptI,*ptI));/* Q1 */
            L_tmp = L_add(L_tmp,L_tmp);/* Q1 */
            L_tmp = Mult_32_16(L_tmp,128);/* Q1 */
            env[i] = L_shl(L_tmp,5);
            move32();/* Q6 */
            ptR++;
            ptI--;
        }

        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            /* itmp[i] += own_random( cng_ener_seed1 )*denv[i]*0.000011f + denv[i]; */
            L_tmp = Mult_32_16(denv[i], Random(cng_ener_seed1));
            L_tmp = Mult_32_16(L_tmp, GAIN_VAR);
            L_tmp = L_add(L_tmp, denv[i]);
            itmp[i] = L_add(L_tmp, itmp[i]);
            move32();/* Q6 */

            if (itmp[i] < 0)
            {
                itmp[i] = L_deposit_l(0);
            }
        }
        ptR = &fft_io[1];
        ptI = &fft_io[sub(L_FFT,1)];
        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            /* *ptR *= sqrt(itmp[i]/env[i]); */
            /* *ptI *= sqrt(itmp[i]/env[i]); */
            L_tmp = L_max(1, itmp[i]); /*Q6*/
            exp = norm_l(L_tmp);
            tmp = extract_h(L_shl(L_tmp, exp));
            exp = sub(31-6, exp);  /* in Q15 (L_tmp in Q6)*/

            exp2 = norm_l(env[i]);
            tmp2 = extract_h(L_shl(env[i], exp2));
            exp2 = sub(31-6, exp2);  /* in Q15 (L_tmp in Q6)*/

            exp = sub(exp2, exp); /* Denormalize and substract */
            if (sub(tmp2, tmp) > 0)
            {
                exp = add(exp, 1);
            }
            if (sub(tmp2, tmp) > 0)
            {
                tmp2 = shr(tmp2, 1);
            }
            tmp = div_s(tmp2, tmp);
            L_tmp = L_deposit_h(tmp);
            L_tmp = Isqrt_lc(L_tmp, &exp); /*Q(31-exp)*/

            L_tmp2 = Mult_32_16(L_tmp,*ptR);/*Q(16-exp)*/
            *ptR = extract_l(L_shr(L_tmp2,sub(sub(16,exp),Q_exc))); /*Q_exc*/
            L_tmp2 = Mult_32_16(L_tmp,*ptI);/*Q(16-exp)*/
            *ptI = extract_l(L_shr(L_tmp2,sub(sub(16,exp),Q_exc))); /*Q_exc*/

            ptR++;
            ptI--;
        }

        ifft_rel_fx(fft_io, L_FFT, LOG2_L_FFT);

        IF ( sub(L_frame,L_FRAME16k) == 0 )
        {
            modify_Fs_fx( fft_io, L_FFT, 12800, fft_io, 16000, exc_mem, 0 );
        }

        /* enr1 = dotp( fft_io, fft_io, L_frame ) / L_frame; */

        enr1 = L_deposit_l(1);
        pt_fft_io = fft_io;
        IF( sub(L_frame, L_FRAME) == 0)
        {
            FOR (j=0; j<128; j++)
            {
                L_tmp = L_mult0(*pt_fft_io, *pt_fft_io);
                pt_fft_io++;
                L_tmp = L_mac0(L_tmp, *pt_fft_io, *pt_fft_io); /* 2*(Q_exc) */
                pt_fft_io++;
                enr1 = L_add(enr1, L_shr(L_tmp, 7)); /* 2*(Q_exc)+1, divide by L_frame done here */
            }
        }
        ELSE /* L_FRAME16k */
        {
            FOR (j=0; j<160; j++)
            {
                L_tmp = L_mult0(*pt_fft_io, *pt_fft_io);
                pt_fft_io++;
                L_tmp = L_mac0(L_tmp, *pt_fft_io, *pt_fft_io); /* 2*(Q_exc) */
                pt_fft_io++;
                enr1 = L_add(enr1, L_shr(Mult_32_16(L_tmp,26214 /* 256/320, Q15 */), 7)); /* 2*(Q_exc)+15+1-16+1, divide by L_frame done here */
            }
        }
        enr1 = L_shr(enr1,sub(add(Q_exc,Q_exc),5));/*Q6*/

        /* add time domain randomization */
        FOR ( i_subfr=0; i_subfr<L_frame; i_subfr += L_SUBFR )
        {

            L_tmp = Mult_32_16(enr1, Random(cng_ener_seed1));
            L_tmp = Mult_32_16(L_tmp, GAIN_VAR);
            L_tmp = L_add(L_tmp, enr1);
            L_tmp = L_max(L_tmp, 1);

            /* enr = dot_product( fft_io, fft_io, L_SUBFR ) + 0.01f */
            tmp = extract_h(Dot_product12(&fft_io[i_subfr], &fft_io[i_subfr], L_SUBFR, &exp));
            exp = add(exp, 8-6);     /* 8 from Q-4, -6 from L_SUBFR */

            /* enr = (float)sqrt( ener_lp*L_SUBFR / enr ) */
            exp2 = norm_l(L_tmp);
            tmp2 = extract_h(L_shl(L_tmp, exp2));
            exp2 = sub(31-6, exp2);  /* in Q15 (L_tmp in Q6)*/

            exp = sub(exp, exp2);

            if (sub(tmp, tmp2) > 0)
            {
                exp = add(exp, 1);
            }
            if (sub(tmp, tmp2) > 0)
            {
                tmp = shr(tmp, 1);
            }
            tmp = div_s(tmp, tmp2);

            L_tmp = L_deposit_h(tmp);

            L_tmp = Isqrt_lc(L_tmp, &exp);/*Q(31-exp)*/

            test();
            test();
            test();
            IF( L_sub(last_core_brate,SID_2k40) != 0 && L_sub(last_core_brate,SID_1k75) != 0 && L_sub(last_core_brate,FRAME_NO_DATA) != 0 && L_sub(core_brate,SID_2k40) == 0 )
            {
                IF ( L_sub(L_tmp,L_shl(1,sub(31,exp))) > 0 )
                {
                    L_tmp = L_shl(1,sub(31,exp));
                }
            }

            tmp = extract_h(L_tmp);

            exp = add(exp, 4);       /* From Q15 to Q19 */
            exp = add(exp, Q_exc);   /* Q_exc+ Q19      */

            FOR (i=0; i<L_SUBFR; i++)
            {
                /* fft_io[i] *= enr */
                L_tmp = L_mult(fft_io[i_subfr+i], tmp); /* Q-4 * Q_exc+19 -> Q_exc +16 */
                fft_io[i_subfr+i] = round_fx(L_shl(L_tmp, exp));/*Q_exc*/
            }
        }

        FOR ( i=0; i<L_frame; i++ )
        {
            /* fft_io[i] = 0.75f*fft_io[i] + exc2[i];*/
            tmp = mult(fft_io[i],24576);
            fft_io[i] = add(tmp,exc2[i]);
            move16();/*Q_exc*/
        }

        /* enr = (dotp( fft_io, fft_io, L_frame ) / L_frame) + 0.01f */

        L_tmp2 = L_deposit_l(1);
        pt_fft_io = fft_io;
        IF( sub(L_frame, L_FRAME) == 0)
        {
            FOR (j=0; j<128; j++)
            {
                L_tmp = L_mult0(*pt_fft_io, *pt_fft_io);
                pt_fft_io++;
                L_tmp = L_mac0(L_tmp, *pt_fft_io, *pt_fft_io); /* 2*(Q_exc) */
                pt_fft_io++;
                L_tmp2 = L_add(L_tmp2, L_shr(L_tmp, 7)); /* 2*(Q_exc)+1, divide by L_frame done here */
            }
        }
        ELSE /* L_FRAME16k */
        {
            FOR (j=0; j<160; j++)
            {
                L_tmp = L_mult0(*pt_fft_io, *pt_fft_io);
                pt_fft_io++;
                L_tmp = L_mac0(L_tmp, *pt_fft_io, *pt_fft_io); /* 2*(Q_exc) */
                pt_fft_io++;
                L_tmp2 = L_add(L_tmp2, L_shr(Mult_32_16(L_tmp,26214 /* 256/320, Q15 */), 7)); /* 2*(Q_exc)+15+1-16+1, divide by L_frame done here */
            }
        }
        L_tmp2 = L_shr(L_tmp2,sub(add(Q_exc,Q_exc),5));/*Q6*/


        /*  enr = (*lp_ener)/enr; */
        /*  ftmp = sqrt(enr); */
        L_tmp = L_max(1, *lp_ener); /*Q6*/
        exp = norm_l(L_tmp);
        tmp = extract_h(L_shl(L_tmp, exp));
        exp = sub(31-6, exp);  /* in Q15 (L_tmp in Q6)*/

        exp2 = norm_l(L_tmp2);
        tmp2 = extract_h(L_shl(L_tmp2, exp2));
        exp2 = sub(31-6, exp2);  /* in Q15 (L_tmp in Q6)*/

        exp = sub(exp2, exp); /* Denormalize and substract */
        if (sub(tmp2, tmp) > 0)
        {
            exp = add(exp, 1);
        }
        if (sub(tmp2, tmp) > 0)
        {
            tmp2 = shr(tmp2, 1);
        }
        tmp = div_s(tmp2, tmp);
        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp); /*Q(31-exp)*/

        IF ( L_sub(L_tmp,L_shl(1,sub(31,exp))) > 0 )
        {
            L_tmp = L_shl(1,sub(31,exp));
        }

        ftmp = extract_l(L_shr(L_tmp,sub(16,exp)));/* Q15 */

        FOR (i=0; i<L_frame; i++)
        {
            /* fft_io[i] *= ftmp;*/
            fft_io[i] = mult(fft_io[i],ftmp);
            move16();/* Q_exc */
        }
        Copy( fft_io, exc2, L_frame );
    }
    IF ( sub(Opt_AMR_WB,1) != 0 )
    {
        Copy( exc3, exc, L_frame );
    }
    ELSE
    {
        Copy( exc2, exc, L_frame );
    }

    IF( sub(L_frame,L_FRAME) == 0)
    {
        interp_code_5over2_fx( exc2, bwe_exc, L_FRAME );
    }
    ELSE
    {
        interp_code_4over2_fx( exc2, bwe_exc, L_frame );
    }
    return;
}

/*-------------------------------------------------------*
 * cng_params_postupd_fx
 *
 * Post-update of CNG parameters
 *-------------------------------------------------------*/
void cng_params_postupd_fx(
    const Word16 ho_circ_ptr,           /* i  : pointer for CNG averaging buffers                  Q0    */
    Word16 *cng_buf_cnt,          /* i/o: counter for CNG store buffers                      Q0    */
    const Word16 *const cng_exc2_buf,   /* i  : Excitation buffer                                  Q_exc */
    const Word16 *const cng_Qexc_buf,   /* i  : Q_exc buffer                                       Q0    */
    const Word32 *const cng_brate_buf,  /* i  : bit rate buffer                                    Q0    */
    Word32 ho_env_circ[]          /* i/o: Envelope buffer                                          */
)
{
    Word16 i, j;
    Word16 Q_exc;
    const Word16 *exc2;
    Word16 fft_io[L_FFT];
    Word32 sp[129];
    Word16 *ptR,*ptI;
    Word32 env[NUM_ENV_CNG];
    Word32 L_tmp;
    Word16 tmp;
    Word16 temp_lo_fx, temp_hi_fx;
    Word16 exp_pow;
    Word16 exp1;
    Word16 CNG_mode;
    Word16 ptr;
    Word32 last_active_brate;

    ptr = add( sub(ho_circ_ptr, *cng_buf_cnt), 1);
    if( ptr < 0 )
    {
        ptr = add(ptr, HO_HIST_SIZE);
    }

    FOR( j = 0; j < *cng_buf_cnt; j++ )
    {
        exc2  = &cng_exc2_buf[ptr*L_FFT];
        Q_exc = cng_Qexc_buf[ptr];
        last_active_brate = cng_brate_buf[ptr];

        /* calculate the spectrum of residual signal */
        Copy(exc2, fft_io, L_FFT);

        fft_rel_fx(fft_io, L_FFT, LOG2_L_FFT);

        ptR = &fft_io[1];
        ptI = &fft_io[L_FFT-1];
        FOR (i=0; i<NUM_ENV_CNG; i++)
        {
            /* sp[i] = 2.0f*(*ptR * *ptR + *ptI * *ptI)/L_FFT; */
            L_tmp = L_mult(*ptR,*ptR);/* 2*Q_exc+1 */
            L_tmp = L_add(L_tmp,L_mult(*ptI,*ptI));/* 2*Q_exc+1 */
            L_tmp = L_add(L_tmp,L_tmp);/* 2*Q_exc+1 */
            L_tmp = Mult_32_16(L_tmp,128);/* 2*Q_exc+1 */
            tmp = add(add(Q_exc,Q_exc),1);
            sp[i] = L_shr(L_tmp,sub(tmp,6));
            move32();/* Q6 */

            ptR++;
            ptI--;
        }

        Copy32(sp,env,NUM_ENV_CNG);
        IF( L_sub(last_active_brate,ACELP_13k20) > 0 )
        {
            CNG_mode = 4;
        }
        ELSE IF( L_sub(last_active_brate,ACELP_9k60) > 0 )
        {
            CNG_mode = 3;
        }
        ELSE IF( L_sub(last_active_brate,ACELP_8k00) > 0 )
        {
            CNG_mode = 2;
        }
        ELSE IF( L_sub(last_active_brate,ACELP_7k20) > 0 )
        {
            CNG_mode = 1;
        }
        ELSE
        {
            CNG_mode = 0;
        }

        /* att = 1/pow(2,ENR_ATT_fx[CNG_mode]); */
        L_tmp = L_shl(L_deposit_l(ENR_ATT_fx[CNG_mode]), 8);/* 16 */
        temp_lo_fx = L_Extract_lc(L_tmp, &temp_hi_fx);

        exp_pow = sub(14, temp_hi_fx);
        L_tmp = Pow2(14, temp_lo_fx);        /* Qexp_pow */
        L_tmp = L_shl(L_tmp, sub(13, exp_pow));   /* Q13 */
        tmp = extract_l(L_tmp);/* Q13 */

        exp1 = norm_s(tmp);
        tmp = shl(tmp, exp1);/*Q(exp1+13) */
        tmp = div_s(16384,tmp); /*Q(15+14-exp1-13) */
        tmp = shr(tmp,sub(1,exp1));/* Q15 */

        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            env[i] = Mult_32_16(env[i],tmp);
            move32();
        }

        /* update the circular buffer of old residual envelope */
        Copy32( env, &(ho_env_circ[(ptr)*NUM_ENV_CNG]), NUM_ENV_CNG );

        ptr = add(ptr, 1);
        if(sub(ptr, HO_HIST_SIZE) == 0)
        {
            ptr = 0;
        }
    }

    *cng_buf_cnt = 0;

    return;

}



/*-------------------------------------------------------*
 * cng_params_upd_fx()
 *
 * update CNG parameters
 *-------------------------------------------------------*/
void cng_params_upd_fx(
    const Word16 lsp_new[],        /* i  : LSP aprameters                                     Q15   */
    const Word16 exc2[],           /* i  : current enhanced excitation                        Q_exc */
    const Word16 L_frame,          /* i  : frame length                                       Q0    */
    Word16 *ho_circ_ptr,     /* i/o: pointer for CNG averaging buffers                  Q0    */
    Word32 ho_ener_circ[],   /* o  : energy buffer for CNG averaging                    Q6    */
    Word16 *ho_circ_size,    /* i/o: size of DTX hangover history buffer for averaging  Q0    */
    Word16 ho_lsp_circ[],    /* o  : old LSP buffer for CNG averaging                   Q15   */
    const Word16 Q_exc,            /* i  : Q value of excitation                                    */
    const Word16 enc_dec_flag,     /* i  : Flag indicating encoder or decoder (ENC,DEC)             */
    Word32 ho_env_circ[],    /* i/o: Envelope buffer                                          */
    Word16 *cng_buf_cnt,     /* i/o: Counter of postponed FFT-processing instances            */
    Word16 cng_exc2_buf[],   /* i/o: Excitation buffer                                  Q_exc */
    Word16 cng_Qexc_buf[],   /* i/o: Q_exc buffer                                       Q0    */
    Word32 cng_brate_buf[],  /* i/o: last_active_brate buffer                           Q0    */
    const Word32 last_active_brate /* i  : Last active bit rate                               Q0    */
)
{
    Word32 L_ener, L_tmp;
    Word16 i, j;
    const Word16 *pt_exc2;
    Word16 tmpv, maxv, scale;
    Word16 fft_io[L_FRAME16k];
    Word32 sp[129];
    Word16 *ptR,*ptI;
    Word32 env[NUM_ENV_CNG];
    Word16 exp1;
    Word16 CNG_mode;
    Word16 tmp;
    Word16 temp_lo_fx, temp_hi_fx;
    Word16 exp_pow;


    /* update the pointer to circular buffer of old LSP vectors */
    *ho_circ_ptr = add(*ho_circ_ptr,1);

    if( sub(*ho_circ_ptr, HO_HIST_SIZE) == 0 )
    {
        *ho_circ_ptr = 0;
        move16();
    }

    /* update the circular buffer of old LSP vectors with the new LSP vector */
    Copy( lsp_new, &(ho_lsp_circ[(*ho_circ_ptr)*M]), M );

    /* calculate the residual signal energy */
    /*enr = dotp( exc2, exc2, L_frame ) / L_frame; */

    maxv = 0;
    move16();
    FOR(i = 0; i < L_frame; i++)
    {
        maxv = s_max(maxv, abs_s(exc2[i]));
    }
    scale = norm_s(maxv);

    pt_exc2 = exc2;
    move16();
    L_ener = L_deposit_l(0);
    IF( sub(L_frame, L_FRAME) == 0)
    {
        FOR (j=0; j<128; j++)
        {
            tmpv = shl(*pt_exc2,scale);
            L_tmp = L_mult0(tmpv, tmpv);    /* 2*(Q_exc+scale) */
            pt_exc2++;
            tmpv = shl(*pt_exc2,scale);
            L_tmp = L_mac0(L_tmp, tmpv, tmpv);
            pt_exc2++;
            L_ener = L_add(L_ener, L_shr(L_tmp, 7)); /* Q(2*(Q_exc+scale)+1) ,division by L_frame done here */
        }
    }
    ELSE /* L_FRAME16k */
    {
        FOR (j=0; j<160; j++)
        {
            tmpv = shl(*pt_exc2,scale);
            L_tmp = L_mult0(tmpv, tmpv);    /* 2*(Q_exc+scale) */
            pt_exc2++;
            tmpv = shl(*pt_exc2,scale);
            L_tmp = L_mac0(L_tmp, tmpv, tmpv);
            pt_exc2++;
            L_ener = L_add(L_ener, L_shr( Mult_32_16(L_tmp,26214 /* 256/320, Q15 */), 7)); /* Q(2*(Q_exc+scale)+15+1-16+1) ,division by L_frame done here */
        }
    }
    L_ener = L_shr(L_ener, sub(shl(add(Q_exc,scale),1),5)); /* Q6 (2*(Q_exc+scale)+1-2*(Q_exc+scale)+5) */

    /* update the circular buffer of old energies */
    ho_ener_circ[*ho_circ_ptr] = L_ener;
    move32();

    IF( sub(enc_dec_flag, ENC) == 0 )
    {
        /* Store residual signal for postponed FFT-processing*/
        *cng_buf_cnt = add(*cng_buf_cnt,1);
        if( sub(*cng_buf_cnt, HO_HIST_SIZE) > 0 )
        {
            *cng_buf_cnt = HO_HIST_SIZE;
            move16();
        }
        Copy( exc2, &(cng_exc2_buf[(*ho_circ_ptr)*L_FFT]), L_FFT );
        cng_Qexc_buf[*ho_circ_ptr] = Q_exc;
        move16();
        cng_brate_buf[*ho_circ_ptr] = last_active_brate;
        move16();
    }
    ELSE
    {
        /* calculate the spectrum of residual signal */
        Copy(exc2, fft_io, L_frame);

        fft_rel_fx(fft_io, L_FFT, LOG2_L_FFT);

        ptR = &fft_io[1];
        ptI = &fft_io[L_FFT-1];
        FOR (i=0; i<NUM_ENV_CNG; i++)
        {
            /* sp[i] = 2.0f*(*ptR * *ptR + *ptI * *ptI)/L_FFT; */
            L_tmp = L_mult(*ptR,*ptR);/* 2*Q_exc+1 */
            L_tmp = L_add(L_tmp,L_mult(*ptI,*ptI));/* 2*Q_exc+1 */
            L_tmp = L_add(L_tmp,L_tmp);/* 2*Q_exc+1 */
            L_tmp = Mult_32_16(L_tmp,128);/* 2*Q_exc+1 */
            tmp = add(add(Q_exc,Q_exc),1);
            sp[i] = L_shr(L_tmp,sub(tmp,6));
            move32();/* Q6 */

            ptR++;
            ptI--;
        }

        Copy32(sp,env,NUM_ENV_CNG);
        IF( L_sub(last_active_brate,ACELP_13k20) > 0 )
        {
            CNG_mode = 4;
        }
        ELSE IF( L_sub(last_active_brate,ACELP_9k60) > 0 )
        {
            CNG_mode = 3;
        }
        ELSE IF( L_sub(last_active_brate,ACELP_8k00) > 0 )
        {
            CNG_mode = 2;
        }
        ELSE IF( L_sub(last_active_brate,ACELP_7k20) > 0 )
        {
            CNG_mode = 1;
        }
        ELSE
        {
            CNG_mode = 0;
        }

        /* att = 1/pow(2,ENR_ATT_fx[CNG_mode]); */
        L_tmp = L_shl(L_deposit_l(ENR_ATT_fx[CNG_mode]), 8);/* 16 */
        temp_lo_fx = L_Extract_lc(L_tmp, &temp_hi_fx);

        exp_pow = sub(14, temp_hi_fx);
        L_tmp = Pow2(14, temp_lo_fx);        /* Qexp_pow */
        L_tmp = L_shl(L_tmp, sub(13, exp_pow));   /* Q13 */
        tmp = extract_l(L_tmp);/* Q13 */

        exp1 = norm_s(tmp);
        tmp = shl(tmp, exp1);/*Q(exp1+13) */
        tmp = div_s(16384,tmp); /*Q(15+14-exp1-13) */
        tmp = shr(tmp,sub(1,exp1));/* Q15 */

        FOR ( i=0; i<NUM_ENV_CNG; i++ )
        {
            env[i] = Mult_32_16(env[i],tmp);
            move32();
        }

        /* update the circular buffer of old residual envelope */
        /* Copy32( env, &(ho_env_circ[add(shl(*ho_circ_ptr,4),shl(*ho_circ_ptr,2))]), NUM_ENV_CNG ); */
        Copy32( env, &(ho_env_circ[(*ho_circ_ptr)*NUM_ENV_CNG]), NUM_ENV_CNG );
    }
    *ho_circ_size = add(*ho_circ_size,1);
    if( sub(*ho_circ_size,HO_HIST_SIZE) > 0 )
    {
        *ho_circ_size = HO_HIST_SIZE;
        move16();
    }

    return;
}
