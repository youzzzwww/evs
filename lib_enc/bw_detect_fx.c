/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_enc_fx.h"    /* Encoder static table prototypes        */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

#include "rom_com_fx.h"
#include "basop_util.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define ALPHA_BWD_FX        24576   /* 0.75 in Q15*/
#define BWD_LT_THRESH_FX    19661   /* 0.6 in Q15*/

#define BWD_COUNT_MAX       100
#define BWD_COUNT_WIDER_BW  10

#define BWD_N_BINS_MAX      13

#define CLDFB_ENER_OFFSET_FX  26214 /* 1.6 in Q14 */

/*-------------------------------------------------------------------*
 * bw_detect()
 *
 * WB, SWB and FB bandwidth detector
 *--------------------------------------------------------------------*/

void bw_detect_fx(
    Encoder_State_fx *st,                /* i/o: Encoder State           */
    const Word16 signal_in[],          /* i  : input signal            */
    const Word16 localVAD,
    Word32 *enerBuffer,                /* i  : CLDFB Energy   Q31        */
    Word16 *cldfbBuf_Ener_Exp          /* i  : CLDFB Energy Exponent     */
)
{
    Word16 Q_dct;
    Word16 i, j, k, bw_max, bin_width, n_bins;
    Word16 max_NB, max_WB, max_SWB, max_FB, mean_NB, mean_WB, mean_SWB, mean_FB;    /* Q11*/
    const Word16 *pt, *pt1;
    Word16 spect[BWD_TOTAL_WIDTH], spect_bin[BWD_N_BINS_MAX];
    Word16 in_win16[BWD_TOTAL_WIDTH];
    Word16 e_tmp, f_tmp;
    Word32 L_tmp, sum32;

    Word32 L_tmp1, L_tmp2, L_tmp3;
    Word16 scale;
    Word16 ScalFac, ScalFacInv;
    Word32 cldfb_bin[9];
    Word16 cldfb_bin_Exp[9];
    Word16 cldfb_bin_width = 4;
    const Word32 *pt32;
    Word32 max_NB32, max_WB32, max_SWB32, max_FB32, mean_NB32, mean_WB32, mean_SWB32, mean_FB32;    /* Q11*/  /* we need Word32 for the new cldfb energy vectors */

    /* only for debugging */
    /* float cldfbBuf_Ener_float[CLDFB_NO_CHANNELS_MAX];
      for(i=0; i< CLDFB_NO_CHANNELS_MAX; i++){
        cldfbBuf_Ener_float[i]  = (enerBuffer[i] * pow (2, -(31 - cldfbBuf_Ener_Exp[i])));
      }
    */

    IF( L_sub(st->input_Fs_fx,8000) > 0 )
    {

        IF ( enerBuffer != NULL)
        {
            n_bins = 9;
            move16();
            scale = st->cldfbAna_Fx->scale;
            move16();/* 7Q8 */

            /*ScalFac =  1/ ( st->cldfbAnaEnc->scale * st->cldfbAnaEnc->scale * 8.f);*/
            /*hs->CLDFBscalingFactor = div_s(1, shl(i_mult2(CLDFBscale, CLDFBscale), 3));*/

            assert(FL2WORD16(1.0/(1<<4)) < mult(scale, scale));
            /* Exponent ScalFacInv: -16 = -(2*7 (CLDFBscale) + 2 (8.0) */
            ScalFacInv = shl(mult(scale,scale),1); /* Q8*Q8 = Q16 + shl -> Q17 -16 -> Q1; shl ->  Q2 */
            /* Exponent ScalFac: -15 = -(2*7 (CLDFBscale) + 2 (8.0) - 1 (1.0)) */
            ScalFac = div_s(0x4000,ScalFacInv); /* bin(17214) *2^-15 * 2^-15 = 0.0000160 , use CLDFBscalingFactor_EXP for this*/  /*Q15*/


            /*set_f( cldfb_bin, 0.001f, 9 );*/
            set32_fx( cldfb_bin, 1, n_bins );   /* Q31*/
            set16_fx( cldfb_bin_Exp, -15, n_bins );

            /* NB: 1.2 - 2.8 kHz, 4 cldfb-bands */
            FOR(i=0; i< cldfb_bin_width; i++ )
            {
                cldfb_bin[0] = BASOP_Util_Add_Mant32Exp( cldfb_bin[0],cldfb_bin_Exp[0], enerBuffer[3+i], cldfbBuf_Ener_Exp[3+i], &(cldfb_bin_Exp[0]) );
                move32();/* result: Q31 */
            }

            cldfb_bin[0] = Mpy_32_16_1(cldfb_bin[0], ScalFac );
            move32(); /* Q31 */
            cldfb_bin_Exp[0] = add( cldfb_bin_Exp[0], CLDFBscalingFactor_EXP);
            move16();

            if(cldfb_bin[i] == 0)
            {
                cldfb_bin[i] = L_deposit_l(1);
            }
            L_tmp = BASOP_Util_Log2(cldfb_bin[0]);    /*(log2(660423549*2^(-31))/64)*2^31*/
            L_tmp = L_add(L_tmp,L_shl(L_deposit_l(cldfb_bin_Exp[0]),31-LD_DATA_SCALE)); /* Q25 */
            cldfb_bin[0] = Mpy_32_16_1(L_tmp, FL2WORD16(1.0f/3.3219280948873623478703194294894f));
            move32();/* 1/log2(10) */ /* Q25 */

            /* WB: 4.4 - 7.2 kHz, 8 cldfb-bands, mid band(14) counted twice */
            IF( L_sub(st->input_Fs_fx,16000) >= 0 )
            {
                /*
                cldfb_bin[1] += Sum( &(enerBuffer[11]), cldfb_bin_width );
                cldfb_bin[2] += Sum( &(enerBuffer[14]), cldfb_bin_width );*/
                FOR(i=0; i< cldfb_bin_width; i++ )
                {
                    cldfb_bin[1] = BASOP_Util_Add_Mant32Exp( cldfb_bin[1],cldfb_bin_Exp[1], enerBuffer[11+i], cldfbBuf_Ener_Exp[11+i], &(cldfb_bin_Exp[1]) );
                    move32();
                    cldfb_bin[2] = BASOP_Util_Add_Mant32Exp( cldfb_bin[2],cldfb_bin_Exp[2], enerBuffer[14+i], cldfbBuf_Ener_Exp[14+i], &(cldfb_bin_Exp[2]) );
                    move32();
                }
                FOR(i=1; i<= 2; i++ )
                {
                    cldfb_bin[i] = Mpy_32_16_1(cldfb_bin[i], ScalFac );
                    move32();
                    cldfb_bin_Exp[i] = add( cldfb_bin_Exp[i], CLDFBscalingFactor_EXP);
                    move16();

                    if(cldfb_bin[i] == 0)
                    {
                        cldfb_bin[i] = L_deposit_l(1);
                    }
                    L_tmp = BASOP_Util_Log2(cldfb_bin[i]);    /*(log2(660423549*2^(-31))/64)*2^31*/
                    L_tmp = L_add(L_tmp,L_shl(L_deposit_l(cldfb_bin_Exp[i]),31-LD_DATA_SCALE)); /* Q25 */
                    cldfb_bin[i] = Mpy_32_16_1(L_tmp, FL2WORD16(1.0f/3.3219280948873623478703194294894f));
                    move32();/* 1/log2(10) */ /* Q25 */
                }
            }

            /* SWB: 9.2 - 15.6 kHz, 16 cldfb-bands */
            IF( L_sub(st->input_Fs_fx,32000) >= 0 )
            {
                /*
                cldfb_bin[3] += Sum( &(enerBuffer[23]), cldfb_bin_width );
                cldfb_bin[4] += Sum( &(enerBuffer[27]), cldfb_bin_width );
                cldfb_bin[5] += Sum( &(enerBuffer[31]), cldfb_bin_width );
                cldfb_bin[6] += Sum( &(enerBuffer[35]), cldfb_bin_width );
                 */
                FOR(i=0; i< cldfb_bin_width; i++ )
                {
                    cldfb_bin[3] = BASOP_Util_Add_Mant32Exp( cldfb_bin[3],cldfb_bin_Exp[3], enerBuffer[23+i], cldfbBuf_Ener_Exp[23+i], &(cldfb_bin_Exp[3]) );
                    move32();
                    cldfb_bin[4] = BASOP_Util_Add_Mant32Exp( cldfb_bin[4],cldfb_bin_Exp[4], enerBuffer[27+i], cldfbBuf_Ener_Exp[27+i], &(cldfb_bin_Exp[4]) );
                    move32();
                    cldfb_bin[5] = BASOP_Util_Add_Mant32Exp( cldfb_bin[5],cldfb_bin_Exp[5], enerBuffer[31+i], cldfbBuf_Ener_Exp[31+i], &(cldfb_bin_Exp[5]) );
                    move32();
                    cldfb_bin[6] = BASOP_Util_Add_Mant32Exp( cldfb_bin[6],cldfb_bin_Exp[6], enerBuffer[35+i], cldfbBuf_Ener_Exp[35+i], &(cldfb_bin_Exp[6]) );
                    move32();
                }
                FOR(i=3; i<= 6; i++ )
                {
                    cldfb_bin[i] = Mpy_32_16_1(cldfb_bin[i], ScalFac );
                    move32();
                    cldfb_bin_Exp[i] = add( cldfb_bin_Exp[i], CLDFBscalingFactor_EXP);
                    move16();

                    if(cldfb_bin[i] == 0)
                    {
                        cldfb_bin[i] = L_deposit_l(1);
                    }
                    L_tmp = BASOP_Util_Log2(cldfb_bin[i]);    /*(log2(660423549*2^(-31))/64)*2^31*/
                    L_tmp = L_add(L_tmp,L_shl(L_deposit_l(cldfb_bin_Exp[i]),31-LD_DATA_SCALE)); /* Q25 */
                    cldfb_bin[i] = Mpy_32_16_1(L_tmp, FL2WORD16(1.0f/3.3219280948873623478703194294894f));
                    move32();/* 1/log2(10) */ /* Q25 */
                }
            }

            /* FB:  16.8 - 20.0 kHz, 8 cldfb-bands */
            IF( L_sub(st->input_Fs_fx,48000) >= 0 )
            {
                /*
                cldfb_bin[7] += Sum( &(enerBuffer[42]), cldfb_bin_width );
                cldfb_bin[8] += Sum( &(enerBuffer[46]), cldfb_bin_width );
                 */
                FOR(i=0; i< cldfb_bin_width; i++ )
                {
                    cldfb_bin[7] = BASOP_Util_Add_Mant32Exp( cldfb_bin[7],cldfb_bin_Exp[7], enerBuffer[42+i], cldfbBuf_Ener_Exp[42+i], &(cldfb_bin_Exp[7]) );
                    move32();
                    cldfb_bin[8] = BASOP_Util_Add_Mant32Exp( cldfb_bin[8],cldfb_bin_Exp[8], enerBuffer[46+i], cldfbBuf_Ener_Exp[46+i], &(cldfb_bin_Exp[8]) );
                    move32();
                }
                FOR(i=7; i<= 8; i++ )
                {
                    cldfb_bin[i] = Mpy_32_16_1(cldfb_bin[i], ScalFac );
                    move32();
                    cldfb_bin_Exp[i] = add( cldfb_bin_Exp[i], CLDFBscalingFactor_EXP);
                    move16();

                    if(cldfb_bin[i] == 0)
                    {
                        cldfb_bin[i] = L_deposit_l(1);
                    }
                    L_tmp = BASOP_Util_Log2(cldfb_bin[i]);    /*(log2(660423549*2^(-31))/64)*2^31*/
                    L_tmp = L_add(L_tmp,L_shl(L_deposit_l(cldfb_bin_Exp[i]),31-LD_DATA_SCALE)); /* Q25 */
                    cldfb_bin[i] = Mpy_32_16_1(L_tmp, FL2WORD16(1.0f/3.3219280948873623478703194294894f));
                    move32();/* 1/log2(10) */ /* Q25 */
                }
            }
            /* cldfb_bin_Exp[] are applied now in cldfb_bin[i] -> don't use again */
            set16_fx( cldfb_bin_Exp, 0, n_bins );

        }
        ELSE
        {

            /* set width of a speactral bin (corresponds to 1.5kHz) */
            IF( L_sub(st->input_Fs_fx,16000) == 0 )
            {
                bw_max = WB;
                move16();
                bin_width = 60;
                move16();
                n_bins = 5;
                move16();       /* spectrum to 7.5 kHz */
            }
            ELSE IF( L_sub(st->input_Fs_fx,32000) == 0 )
            {
                bw_max = SWB;
                move16();
                bin_width = 30;
                move16();
                n_bins = 10;
                move16();        /* spectrum to 15.0 kHz */
            }
            ELSE  /* st->input_Fs == 48000 */
            {
                bw_max = FB;
                move16();
                bin_width = 20;
                move16();
                n_bins = BWD_N_BINS_MAX;
                move16();    /* spectrum to 19.5 kHz */
            }

            /*---------------------------------------------------------------------*
             * windowing of the input signal
             *---------------------------------------------------------------------*/

            pt = signal_in;
            pt1 = hann_window_320_fx;
            /* 1st half of the window */
            FOR( i=0; i<BWD_TOTAL_WIDTH/2; i++ )
            {
                /*in_win[i] = *pt++ * *pt1++;*/
                in_win16[i] = mult_r(*pt++,*pt1++);
                move16();     /* Q0*Q15 -> Q16*/
            }
            pt1--;

            /* 2nd half of the window */
            FOR(; i<BWD_TOTAL_WIDTH; i++ )
            {
                /*in_win[i] = *pt++ * *pt1--;*/
                in_win16[i] = mult_r(*pt++,*pt1--);
                move16();
            }

            /*---------------------------------------------------------------------*
             * transform into frequency domain
             *---------------------------------------------------------------------*/

            /*edct( in_win, spect, BWD_TOTAL_WIDTH );*/
            Q_dct  = 0;
            edct_16fx( in_win16, spect, BWD_TOTAL_WIDTH, 6 );
            /*---------------------------------------------------------------------*
             * compute energy per spectral bins
             *---------------------------------------------------------------------*/

            set16_fx( spect_bin, 1, n_bins );
            Q_dct = shl(Q_dct,1);

            FOR( k=0; k<=bw_max; k++ )
            {
                FOR( i=bwd_start_bin_fx[k]; i<=bwd_end_bin_fx[k]; i++ )
                {
                    sum32 = L_deposit_l(1);
                    pt1 = &spect[i_mult2(i,bin_width)];
                    FOR( j=0; j<bin_width; j++ )
                    {
                        sum32 = L_mac0(sum32,*pt1,*pt1);
                        pt1++;
                    }

                    IF( L_sub(sum32,1) <= 0 )
                    {
                        /*deal with zero spectrum*/
                        spect_bin[i] = -1;
                        move16();
                    }
                    ELSE
                    {
                        /* spect_bin[i] = (float)log10(spect_bin[i]);
                                                = log2(spect_bin[i])*log10(2);  */
                        e_tmp = norm_l(sum32);
                        L_tmp = L_shl(sum32, e_tmp);
                        f_tmp = Log2_norm_lc(L_tmp);
                        e_tmp = sub(sub(30,e_tmp),Q_dct);
                        L_tmp = Mpy_32_16(e_tmp, f_tmp, 9864);       /* Q16 */
                        spect_bin[i] = round_fx(L_shl(L_tmp, 11));   /* Q11 */
                    }
                }
            }
        }

        IF ( enerBuffer != NULL )
        {
            Word16 cldfb_ener_offset;
            Word32 cldfb_ener_offset_32;

            pt32 = cldfb_bin;   /* Q25 */
            /* cldfb detections */

            /* NB: 1,6 - 3,2 kHz, 4 cldfb-bands (1 bin)   */
            mean_NB32 = L_add(*pt32++, 0);
            max_NB32 = L_add(mean_NB32, 0);

            /* WB: 4,4 - 7,6 kHz, 8 cldfb-bands (2 bins)  */

            maximum_32_fx( pt32, 2, &max_WB32 );

            L_tmp   = L_shr(*pt32++, 1);
            mean_WB32 = L_add(L_tmp,L_shr(*pt32++, 1)); /* FL2WORD32( 0.5)*/
            /*L_tmp = L_mac(L_tmp,*pt32++,16384);*/
            /*mean_WB = round_fx(L_tmp);*/

            /* Q25 + Q14*/

            cldfb_ener_offset    = (Word16) CLDFB_ENER_OFFSET_FX;                /* Q14 */        move16();
            cldfb_ener_offset_32 = L_deposit_l(cldfb_ener_offset);               /* Q14 in 32bit var */
            cldfb_ener_offset_32 = L_shl(cldfb_ener_offset_32, 25-14 );          /* Q14 -> Q25 */

            mean_NB = extract_l(L_shr( L_add(mean_NB32 ,cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */
            max_NB  = extract_l(L_shr( L_add(max_NB32  ,cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */
            mean_WB = extract_l(L_shr( L_add(mean_WB32 ,cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */
            max_WB  = extract_l(L_shr( L_add(max_WB32  ,cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */


            /*if WB */
            IF( L_sub(st->input_Fs_fx,16000) == 0 )
            {
                /* for 16kHz sampled inputs, do not check SWB & FB */
                mean_SWB = 0;
                move16();
                max_SWB  = 0;
                move16();
                mean_FB  = 0;
                move16();
                max_FB   = 0;
                move16();
            }
            ELSE
            {
                /* else if SWB */
                IF( L_sub(st->input_Fs_fx,32000) == 0 )
                {

                    /* for 32kHz sampled inputs, do not check FB */
                    mean_FB = 0;
                    move16();
                    max_FB  = 0;
                    move16();

                    /*  SWB: 8,8 - 15,2 kHz, 16 cldfb-bands (4 bins) */

                    maximum_32_fx( pt32, 4, &max_SWB32 );

                    L_tmp    = L_shr(*pt32++, 2);  /*  /4 */
                    L_tmp    = L_add(L_tmp,L_shr(*pt32++, 2));
                    L_tmp    = L_add(L_tmp,L_shr(*pt32++, 2));
                    mean_SWB32 = L_add(L_tmp,L_shr(*pt32++, 2));

                    mean_SWB  = extract_l(L_shr(L_add(mean_SWB32, cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */
                    max_SWB   = extract_l(L_shr(L_add(max_SWB32 , cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */

                }
                ELSE
                { /* FB */
                    /*  SWB: 8,8 - 15,2 kHz, 16 cldfb-bands (4 bins) */

                    maximum_32_fx( pt32, 4, &max_SWB32 );

                    L_tmp    = L_shr(*pt32++, 2);  /*  /4 */
                    L_tmp    = L_add(L_tmp,L_shr(*pt32++, 2));
                    L_tmp    = L_add(L_tmp,L_shr(*pt32++, 2));
                    mean_SWB32 = L_add(L_tmp,L_shr(*pt32++, 2));

                    mean_SWB  = extract_l(L_shr(L_add(mean_SWB32, cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */
                    max_SWB   = extract_l(L_shr(L_add(max_SWB32 , cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */


                    /*  FB: 16,4 - 19,6 kHz, 8 cldfb-bands (2 bins) */

                    maximum_32_fx( pt32, 2, &max_FB32 );

                    L_tmp    = L_shr(*pt32++, 1);
                    mean_FB32  = L_add(L_tmp,L_shr(*pt32++, 1));

                    mean_FB  = extract_l(L_shr(L_add(mean_FB32, cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */
                    max_FB   = extract_l(L_shr(L_add(max_FB32 , cldfb_ener_offset_32), 25-11));  /* (Q25 + Q25) -> Q11 */

                }
            }
        }
        ELSE
        {
            pt = (const Word16 *)spect_bin;
            /* NB:  1.5-3.0kHz (1 bin)   */
            pt++;
            mean_NB = *pt++;
            move16();
            max_NB = mean_NB;
            move16();

            /* WB:  4.5-7.5kHz (2 bins)  */
            pt++;
            maximum_fx( pt, 2, &max_WB );
            L_tmp = L_mult(*pt++,16384);
            L_tmp = L_mac(L_tmp,*pt++,16384);
            mean_WB = round_fx(L_tmp);

            IF( L_sub(st->input_Fs_fx,16000) == 0 )
            {
                /* for 16kHz sampled inputs, do not check SWB & FB */
                mean_SWB = 0;
                move16();
                max_SWB = 0;
                move16();
                mean_FB = 0;
                move16();
                max_FB = 0;
                move16();
            }
            ELSE
            {
                /* SWB: 9.0-15.0kHz (4 bins) */
                pt++;
                maximum_fx( pt, 4, &max_SWB );
                L_tmp = L_mult(*pt++,8192);
                L_tmp = L_mac(L_tmp,*pt++,8192);
                L_tmp = L_mac(L_tmp,*pt++,8192);
                L_tmp = L_mac(L_tmp,*pt++,8192);
                mean_SWB = round_fx(L_tmp);

                IF( L_sub(st->input_Fs_fx,48000) == 0 )
                {
                    /* FB: 16.5-19.5kHz (2 bins) */
                    pt++;
                    maximum_fx( pt, 2, &max_FB );
                    L_tmp = L_mult(*pt++,16384);
                    L_tmp = L_mac(L_tmp,*pt++,16384);
                    mean_FB = round_fx(L_tmp);
                }
                ELSE
                {
                    /* for 32kHz sampled inputs, do not check FB */
                    mean_FB = 0;
                    move16();
                    max_FB = 0;
                    move16();
                }
            }
        }
        /*---------------------------------------------------------------------*
         * update LT counters and energies
         *---------------------------------------------------------------------*/

        /*if( localVAD || st->lp_noise > 30 )*/
        test();
        IF( localVAD  || sub(st->lp_noise_fx,7680) > 0 )
        {
            /*st->lt_mean_NB_fx = ALPHA_BWD * st->lt_mean_NB_fx + (1-ALPHA_BWD) * mean_NB;*/
            L_tmp = L_mult(ALPHA_BWD_FX, st->lt_mean_NB_fx);                                 /* Q15 * Q11 -> Q27 */
            L_tmp = L_mac(L_tmp, 32768-ALPHA_BWD_FX, mean_NB);                               /* Q15 * Q11) -> L_mac(Q27, Q27) -> Q27*/
            st->lt_mean_NB_fx = round_fx(L_tmp);                                             /* Q11 (27-16) */

            /*st->lt_mean_WB_fx = ALPHA_BWD * st->lt_mean_WB_fx + (1-ALPHA_BWD) * mean_WB;*/
            L_tmp = L_mult(ALPHA_BWD_FX, st->lt_mean_WB_fx);
            L_tmp = L_mac(L_tmp, 32768-ALPHA_BWD_FX, mean_WB);
            st->lt_mean_WB_fx = round_fx(L_tmp);

            /*st->lt_mean_SWB_fx = ALPHA_BWD * st->lt_mean_SWB_fx + (1-ALPHA_BWD) * mean_SWB;*/
            L_tmp = L_mult(ALPHA_BWD_FX, st->lt_mean_SWB_fx);
            L_tmp = L_mac(L_tmp, 32768-ALPHA_BWD_FX, mean_SWB);
            st->lt_mean_SWB_fx = round_fx(L_tmp);



            IF ( enerBuffer != NULL )
            {
                /*if( 0.9f * max_WB > BWD_LT_THRESH_FX * st->lt_mean_NB_fx )*/

                /* optim: if(  max_WB > (BWD_LT_THRESH_FX / 0.9) * st->lt_mean_NB_fx )*/

                L_tmp = L_mult(3686,max_WB);   /* (0.9 in Q12) x Q11 -> Q24*/

                L_tmp1 = L_mult(BWD_LT_THRESH_FX,st->lt_mean_NB_fx);  /* Q15 x Q11 -> Q27 */
                L_tmp1 = L_shl(L_tmp1, 24-27);                        /* Q27 -> Q24) */
                L_tmp1 = L_sub(L_tmp, L_tmp1 );

                IF( L_tmp1 > 0 )
                {
                    /*if( 2.5f * max_WB > max_NB )*/
                    L_tmp  = L_mult(10240,max_WB);  /* 2.5 in Q12 x Q11 -> Q24 */
                    L_tmp1 = L_mult(max_NB, 4096);  /* Q11 x (1 in Q12) -> Q24 */
                    IF( L_sub(L_tmp,L_tmp1) > 0 )   /* Q24 */
                    {
                        st->count_WB_fx = add(st->count_WB_fx,1);
                    }
                }
                ELSE
                {
                    /*if( 3.5f * mean_WB < mean_NB )*/
                    L_tmp =  L_mult(14336,mean_WB);   /* 3.5 in Q12 x Q11 -> Q24*/
                    L_tmp1 = L_mult(mean_NB,4096);
                    L_tmp = L_sub(L_tmp, L_tmp1);
                    IF( L_tmp < 0 )
                    {
                        st->count_WB_fx = sub(st->count_WB_fx, 1);
                    }
                }

                /*if( 0.83f * max_SWB > BWD_LT_THRESH_FX * st->lt_mean_WB_fx   &&   max_WB > BWD_LT_THRESH_FX * st->lt_mean_NB_fx )*/
                /*    IF( L_msu( L_tmp,BWD_LT_THRESH_FX,st->lt_mean_WB_fx) > 0 && L_msu( L_deposit_h(max_WB),BWD_LT_THRESH_FX,st->lt_mean_NB_fx) > 0 )
                    {*/

                L_tmp = L_mult(3400,max_SWB);   /* (0.83 in Q12) x Q11 -> Q24*/
                L_tmp1 = L_mult(BWD_LT_THRESH_FX,st->lt_mean_WB_fx); /* Q15 x Q11 -> Q27 */
                L_tmp1 = L_shl(L_tmp1, 24-27); /* Q27 -> Q24) */
                L_tmp1 = L_sub(L_tmp, L_tmp1 );

                L_tmp  = L_mult( max_WB,4096);
                L_tmp2 = L_mult(BWD_LT_THRESH_FX,st->lt_mean_NB_fx); /* Q15 x Q11 -> Q27 */
                L_tmp2 = L_shl(L_tmp2, 24-27); /* Q27 -> Q24) */
                L_tmp2 = L_sub(L_tmp, L_tmp2 );

                test();
                IF(L_tmp1 > 0 && L_tmp2 > 0 )
                {
                    /*if( 2 * max_SWB > max_WB )*/
                    L_tmp  = L_mult(max_WB,4096);
                    L_tmp1 = L_mult(8192,max_SWB);      /* 2.0 in Q12 x Q11 -> Q24*/
                    L_tmp1 = L_sub(L_tmp1, L_tmp);  /* Q24 - (Q11 x (1 in Q12) ) = Q24 */
                    IF(L_tmp1 > 0 )
                    {
                        st->count_SWB_fx = add(st->count_SWB_fx, 1);
                    }
                }
                ELSE
                {
                    /*if( 3 * mean_SWB < mean_WB )*/
                    L_tmp =  L_mult(mean_WB,4096);
                    L_tmp1 = L_mult(12288,mean_SWB);     /* 3.0 in Q12 x Q11 -> Q24*/
                    L_tmp1 = L_sub(L_tmp1,L_tmp);  /* Q24 - (Q11 x (1 in Q12) ) = Q24 */
                    IF(L_tmp1 < 0 )
                    {
                        st->count_SWB_fx = sub(st->count_SWB_fx,1);
                    }
                }
                /*if( max_FB > BWD_LT_THRESH_FX * st->lt_mean_SWB_fx && 0.83f * max_SWB > BWD_LT_THRESH_FX * st->lt_mean_WB_fx && max_WB > BWD_LT_THRESH_FX * st->lt_mean_NB_fx )*/

                L_tmp  = L_mult(max_FB, 4096);                          /* Q11 x (1 in Q12) = Q24 */
                L_tmp1 = L_mult(BWD_LT_THRESH_FX, st->lt_mean_SWB_fx);  /* Q15 x Q11 -> Q27 */
                L_tmp1 = L_shl(L_tmp1, 24-27);                          /* Q27 -> Q24) */
                L_tmp1 = L_sub( L_tmp, L_tmp1);                          /* Q24 */

                L_tmp  = L_mult(max_SWB, 3400 );                      /* (0.83 in Q12) x Q11) =  Q24 */
                L_tmp2 = L_mult(BWD_LT_THRESH_FX,st->lt_mean_WB_fx);  /* Q15 x Q11 -> Q27 */
                L_tmp2 = L_shl(L_tmp2, 24-27);                        /* Q27 -> Q24) */
                L_tmp2 = L_sub(L_tmp, L_tmp2 );                       /* Q24 */

                L_tmp = L_mult(max_WB,4096); /* Q11 x (1 in Q12) = Q24*/

                L_tmp3 = L_mult(BWD_LT_THRESH_FX,st->lt_mean_NB_fx); /* Q15 x Q11 -> Q27 */
                L_tmp3 = L_shl(L_tmp3, 24-27); /* Q27 -> Q24) */
                L_tmp3 = L_sub(L_tmp, L_tmp3 );

                test();
                test();
                IF(L_tmp1 > 0 && L_tmp2 > 0 && L_tmp3 > 0 )
                {
                    /*if( 3 * max_FB > max_SWB )*/
                    L_tmp  = L_mult( max_SWB,4096 );     /* Q11 x (1 in Q12) = Q24*/
                    L_tmp1 = L_mult(12288,max_FB);       /* 3.0 in Q12 x Q11 -> Q24*/
                    L_tmp1 = L_sub(L_tmp1, L_tmp);       /* Q24 */
                    IF(L_tmp1 > 0 )
                    {
                        st->count_FB_fx = add(st->count_FB_fx,1);
                    }
                }
                ELSE
                {
                    /* if( 4.1f * mean_FB < mean_SWB )*/
                    L_tmp  = L_mult(mean_SWB,4096);       /* Q11 x (1 in Q12) = Q24 */
                    L_tmp1 = L_mult(16794,mean_FB);       /* 4.1 in Q12 x Q11 -> Q24*/
                    L_tmp1 = L_sub(L_tmp1,L_tmp);         /* Q24 */
                    IF(L_tmp1 < 0 )
                    {
                        st->count_FB_fx = sub(st->count_FB_fx,1);
                    }
                }

            }
            ELSE  /* DCT based detection */
            {
                /*if( max_WB > BWD_LT_THRESH * st->lt_mean_NB_fx )*/
                IF( L_msu( L_deposit_h(max_WB),BWD_LT_THRESH_FX,st->lt_mean_NB_fx) > 0 )
                {
                    /*if( 2 * max_WB > max_NB )*/
                    L_tmp = L_mult(8192,max_WB);   /* 2.0 in Q12 x Q11 -> Q24*/
                    if( L_msu(L_tmp,max_NB,4096) > 0 )
                    {
                        st->count_WB_fx = add(st->count_WB_fx,1);
                    }
                }
                ELSE
                {
                    /*if( 2.6f * mean_WB < mean_NB )*/
                    L_tmp = L_mult(10650,mean_WB);   /* 2.6 in Q12 x Q11 -> Q24*/
                    L_tmp = L_msu(L_tmp,mean_NB,4096);
                    test();
                    test();
                    if( L_tmp < 0 && !(sub(mean_WB,-1) == 0 && sub(mean_NB,-1) == 0) )
                    {
                        st->count_WB_fx = sub(st->count_WB_fx, 1);
                    }
                }

                test();
                /*if( max_SWB > BWD_LT_THRESH * st->lt_mean_WB_fx && max_WB > BWD_LT_THRESH * st->lt_mean_NB_fx )*/
                test();
                IF( L_msu( L_deposit_h(max_SWB),BWD_LT_THRESH_FX,st->lt_mean_WB_fx) > 0 && L_msu( L_deposit_h(max_WB),BWD_LT_THRESH_FX,st->lt_mean_NB_fx) > 0 )
                {
                    /*if( 2 * max_SWB > max_WB )*/
                    L_tmp = L_mult(8192,max_SWB);   /* 2.0 in Q12 x Q11 -> Q24*/
                    if( L_msu(L_tmp,max_WB,4096) > 0 )
                    {
                        st->count_SWB_fx = add(st->count_SWB_fx,1);
                    }
                }
                ELSE
                {
                    /*if( 3 * mean_SWB < mean_WB )*/
                    L_tmp = L_mult(12288,mean_SWB);   /* 3.0 in Q12 x Q11 -> Q24*/
                    L_tmp = L_msu(L_tmp,mean_WB,4096);
                    test();
                    test();
                    if( L_tmp < 0 && !(sub(mean_SWB,-1) == 0 && sub(mean_WB,-1) == 0) )
                    {
                        st->count_SWB_fx = sub(st->count_SWB_fx,1);
                    }
                }

                test();
                test();
                /*if( max_FB > BWD_LT_THRESH * st->lt_mean_SWB_fx && max_SWB > BWD_LT_THRESH * st->lt_mean_WB_fx && max_WB > BWD_LT_THRESH * st->lt_mean_NB_fx )*/
                IF( L_msu( L_deposit_h(max_FB),BWD_LT_THRESH_FX,st->lt_mean_SWB_fx) > 0 && L_msu( L_deposit_h(max_SWB),BWD_LT_THRESH_FX,st->lt_mean_WB_fx) > 0 && L_msu( L_deposit_h(max_WB),BWD_LT_THRESH_FX,st->lt_mean_NB_fx) > 0 )
                {
                    /*if( 2 * max_FB > max_SWB )*/
                    L_tmp = L_mult(8192,max_FB);   /* 2.0 in Q12 x Q11 -> Q24*/
                    if( L_msu(L_tmp,max_SWB,4096) > 0 )
                    {
                        st->count_FB_fx = add(st->count_FB_fx,1);
                    }
                }
                ELSE
                {
                    /*if( 3 * mean_FB < mean_SWB )*/
                    L_tmp = L_mult(12288,mean_FB);   /* 3.0 in Q12 x Q11 -> Q24*/
                    test();
                    test();
                    if( L_msu(L_tmp,mean_SWB,4096) < 0  && !(sub(mean_FB,-1) == 0 && sub(mean_SWB,-1) == 0) )
                    {
                        st->count_FB_fx = sub(st->count_FB_fx,1);
                    }
                }
            }

            st->count_WB_fx  = s_min(st->count_WB_fx,BWD_COUNT_MAX);
            move16();
            st->count_SWB_fx = s_min(st->count_SWB_fx,BWD_COUNT_MAX);
            move16();
            st->count_FB_fx  = s_min(st->count_FB_fx,BWD_COUNT_MAX);
            move16();
            st->count_WB_fx  = s_max(st->count_WB_fx,0);
            move16();
            st->count_SWB_fx = s_max(st->count_SWB_fx,0);
            move16();
            st->count_FB_fx  = s_max(st->count_FB_fx,0);
            move16();

            /*---------------------------------------------------------------------*
             * check against thresholds
             * detect a band-width change
             *---------------------------------------------------------------------*/

            /* switching to a higher BW */
            IF( sub(st->last_input_bwidth_fx,NB) == 0 )
            {
                IF( sub(st->count_WB_fx,BWD_COUNT_WIDER_BW) > 0 )
                {
                    st->input_bwidth_fx = WB;
                    move16();
                    st->count_WB_fx = BWD_COUNT_MAX;
                    move16();

                    IF( sub(st->count_SWB_fx,BWD_COUNT_WIDER_BW) > 0 )
                    {
                        st->input_bwidth_fx = SWB;
                        move16();
                        st->count_SWB_fx = BWD_COUNT_MAX;
                        move16();

                        IF( sub(st->count_FB_fx,BWD_COUNT_WIDER_BW) > 0 )
                        {
                            st->input_bwidth_fx = FB;
                            move16();
                            st->count_FB_fx = BWD_COUNT_MAX;
                            move16();
                        }
                    }
                }
            }

            test();
            IF( sub(st->last_input_bwidth_fx,WB) == 0 && L_sub(st->input_Fs_fx,16000) > 0 )
            {
                IF( sub(st->count_SWB_fx,BWD_COUNT_WIDER_BW) > 0 )
                {
                    st->input_bwidth_fx = SWB;
                    move16();
                    st->count_SWB_fx = BWD_COUNT_MAX;
                    move16();

                    IF( sub(st->count_FB_fx,BWD_COUNT_WIDER_BW) > 0 )
                    {
                        st->input_bwidth_fx = FB;
                        move16();
                        st->count_FB_fx = BWD_COUNT_MAX;
                        move16();
                    }
                }
            }

            test();
            IF( sub(st->last_input_bwidth_fx,SWB) == 0 && L_sub(st->input_Fs_fx,32000) > 0 )
            {
                IF( sub(st->count_FB_fx,BWD_COUNT_WIDER_BW) > 0 )
                {
                    st->input_bwidth_fx = FB;
                    move16();
                    st->count_FB_fx = BWD_COUNT_MAX;
                    move16();
                }
            }

            /* switching to a lower BW */
            IF( sub(st->last_input_bwidth_fx,FB) == 0 )
            {
                IF( sub(st->count_FB_fx,10) < 0 )
                {
                    st->input_bwidth_fx = SWB;
                    move16();
                    st->count_FB_fx = 0;
                    move16();
                }
                IF( sub(st->count_SWB_fx,10) < 0 )
                {
                    st->input_bwidth_fx = WB;
                    move16();
                    st->count_SWB_fx = 0;
                    move16();
                    st->count_FB_fx = 0;
                    move16();
                }
                IF( sub(st->count_WB_fx,10) < 0 )
                {
                    st->input_bwidth_fx = NB;
                    move16();
                    st->count_WB_fx = 0;
                    move16();
                    st->count_SWB_fx = 0;
                    move16();
                    st->count_FB_fx = 0;
                    move16();
                }
            }

            IF( sub(st->last_input_bwidth_fx,SWB) == 0 )
            {
                IF( sub(st->count_SWB_fx,10) < 0 )
                {
                    st->input_bwidth_fx = WB;
                    move16();
                    st->count_SWB_fx = 0;
                    move16();
                    st->count_FB_fx = 0;
                    move16();
                }
                IF( sub(st->count_WB_fx,10) < 0 )
                {
                    st->input_bwidth_fx = NB;
                    move16();
                    st->count_WB_fx = 0;
                    move16();
                    st->count_SWB_fx = 0;
                    move16();
                    st->count_FB_fx = 0;
                    move16();
                }

            }

            IF( sub(st->last_input_bwidth_fx,WB) == 0 )
            {
                IF( sub(st->count_WB_fx,10) < 0 )
                {
                    st->input_bwidth_fx = NB;
                    move16();
                    st->count_WB_fx = 0;
                    move16();
                    st->count_SWB_fx = 0;
                    move16();
                    st->count_FB_fx = 0;
                    move16();
                }
            }
        }
    }


    /* verify that maximum encoded bandwidth (specified on the command line) is not exceeded */
    IF( sub(st->input_bwidth_fx,st->max_bwidth_fx) > 0 )
    {
        st->input_bwidth_fx = st->max_bwidth_fx;
        move16();
    }

    /* Set and limit the encoded bandwidth */
    IF ( sub(st->codec_mode,MODE1) == 0 )
    {
        Word32 total_brate_fx;

        st->bwidth_fx = st->input_bwidth_fx;
        move16();

        total_brate_fx = L_add(st->total_brate_fx, 0);

        /* change the encoded bandwidth, if not supported at particular bitrate */
        test();
        test();
        test();
        test();
        test();
        IF ( L_sub(total_brate_fx, ACELP_9k60) <= 0 && sub(st->bwidth_fx,NB) != 0 && sub(st->bwidth_fx,WB) != 0 )
        {
            st->bwidth_fx = WB;
            move16();
        }
        ELSE IF ( L_sub(st->total_brate_fx,ACELP_13k20) >= 0 && L_sub(st->total_brate_fx,ACELP_16k40) <= 0 && sub(st->bwidth_fx,SWB) > 0 )
        {
            st->bwidth_fx = SWB;
            move16();
        }
        ELSE IF ( L_sub(st->total_brate_fx,ACELP_32k) >= 0 && sub(st->bwidth_fx,WB) < 0 )
        {
            st->bwidth_fx = WB;
            move16();
        }
    }
    ELSE
    {
        Word16 n, bits_frame_nominal;

        Word32 L_tmp;
        UWord16 lsb;
        Word16 tmpbandwidthMin;

        Mpy_32_16_ss(st->total_brate_fx, 5243, &L_tmp, &lsb); /* 5243 is 1/50 in Q18. (0+18-15=3) */
        bits_frame_nominal = extract_l(L_shr(L_tmp, 3)); /* Q0 */

        FOR (n=0; n<FRAME_SIZE_NB; n++)
        {
            IF (sub(FrameSizeConfig[n].frame_bits,bits_frame_nominal) == 0 )
            {
                BREAK;
            }
        }
        if (n==FRAME_SIZE_NB)
        {
            assert(!"Bitrate not supported: not part of EVS");
        }
        tmpbandwidthMin = FrameSizeConfig[n].bandwidth_min;
        if( sub(st->rf_mode,1) == 0 )
        {
            tmpbandwidthMin = WB;
        }
        st->bwidth_fx = s_max(s_min(st->input_bwidth_fx, FrameSizeConfig[n].bandwidth_max), tmpbandwidthMin);
    }

    return;
}

