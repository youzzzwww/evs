/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "rom_com_fx.h"
#include "prot_fx.h"
#include "stl.h"


/*===================================================================*/
/* FUNCTION      :  dequantize_uvg_fx()                              */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  This function returns the  quantized gain
                    vector given the indices in the gain
                    quantization tables                              */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*         _ (Word16)       iG1  : index into UVG1CB_fx table (Q0)   */
/*         _ (Word16*)      iG2  : indices into UVG2CB_fx (Q0)       */
/*         - (Word32)       Fs   : output sampling rate              */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*         _ (Word16*)      G    : Output quantized gain vector      */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                            _ None.                                */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*                            _ None.                                */
/*===================================================================*/
Word16 dequantize_uvg_fx(
    Word16 iG1,
    Word16 *iG2,
    Word16 *G,
    Word16 bwidth_fx
    ,Word16 do_scale
)
{
    Word16 i, k;
    const Word16 (*UVG1CB)[2]=NULL;
    const Word16 (*UVG2CB1)[5]=NULL;
    const Word16 (*UVG2CB2)[5]=NULL;
    Word16 frac, exp, sc;
    Word32 L_tmp;
    Word16 Q_gain = 0;

    IF( sub(bwidth_fx,NB) == 0 )
    {
        UVG1CB = UVG1CB_NB_FX;
        move16();
        UVG2CB1 = UVG2CB1_NB_FX;
        move16();
        UVG2CB2 = UVG2CB2_NB_FX;
        move16();
    }
    ELSE IF( sub(bwidth_fx,WB) == 0 || sub(bwidth_fx,SWB) == 0)
    {
        test();
        UVG1CB = UVG1CB_WB_FX;
        move16();
        UVG2CB1 = UVG2CB1_WB_FX;
        move16();
        UVG2CB2 = UVG2CB2_WB_FX;
        move16();
    }

    IF ( !do_scale)
    {
        sc = 11;
        move16();
    }
    ELSE
    {
        test();
        IF ( ( sub(UVG1CB[iG1][0], 4096)  < 0 ) && ( sub(UVG1CB[iG1][1],4096) < 0 ) ) /* if x < 1, where 10^x is used for gain computation */
        {
            sc = 8;
            move16();
            Q_gain = 3;
            move16();
        }
        ELSE
        {
            sc = 11;
            move16();
        }
    }

    FOR (i=0; i<2; i++)
    {
        FOR (k=0; k<5; k++)
        {
            IF( i==0 )
            {
                /* pow(10.0, UVG1CB[iG1][i]) = pow(2.0,UVG1CB[iG1][i]*3.321928 */
                L_tmp = L_mult(UVG1CB[iG1][i],27213); /* Q(13+13+1)->Q27 */
                L_tmp = L_shr_r(L_tmp,11); /* Q16 */
                frac = L_Extract_lc(L_tmp,&exp);
                frac = extract_l(Pow2(14,frac));
                G[i*5+k] = round_fx(L_shl(L_mult(frac,UVG2CB1[iG2[i]][k]),exp-sc)); /* Q0 */
            }
            ELSE IF (sub(i,1)==0)
            {
                L_tmp = L_mult(UVG1CB[iG1][i],27213); /* Q(13+13+1)->Q27 */
                L_tmp = L_shr_r(L_tmp,11); /* Q16 */
                frac = L_Extract_lc(L_tmp,&exp);
                frac = extract_l(Pow2(14,frac));
                G[i*5+k] = round_fx(L_shl(L_mult(frac,UVG2CB2[iG2[i]][k]),exp-sc)); /* Q0 */
            }
        }
    }
    return Q_gain;
}

/*===================================================================*/
/* FUNCTION      :  generate_nelp_excitation_fx                      */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  This function computes the random
                    excitation scaled by gain                        */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */

/*         _ (Word16*)      Gains :  Gain vector (Q_exc)                */
/*         _ (Word16)       gain_fac : gain factor (Q14)             */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*         _ (Word16*)      seed  : Random seed (Q0)                 */
/*         _ (Word16*)      output : excitation output (Q_exc)          */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                            _ None.                                */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*                            _ None.                                */
/*===================================================================*/
void generate_nelp_excitation_fx(
    Word16 *seed,            /* i/o: random number seed    */
    Word16 *Gains,           /* i  : excitation gains      Q_exc*/
    Word16 *output,          /* o  : excitation output     */
    Word16 gain_fac          /* i  : gain factor           */
)
{
    Word16 i, len, j;
    Word16 tmp[31], tmp1[31], tmpf, L16;
    Word16 k1, k2, I[31], tmpi;
    Word32 L32;
    Word16 cnt;

    FOR (i=0; i<10; i++)
    {
        IF (sub(i,9)==0)
        {
            len=31;
            move16();
            cnt=8;
            move16();
        }
        ELSE
        {
            len=25;
            move16();
            cnt=6;
            move16();
        }

        FOR (j=0; j<len; j++)
        {
            L32 = L_mult0(*seed,0x0209);   /* L32 = *seed*521; */

            L16 = extract_l(L_add(L32,259));
            *seed = L16;
            move16();               /* Q0 */
            tmp[j] = *seed;
            move16();               /* Q15, tmp[j]=*seed/32768 */

            tmp1[j] = abs_s(tmp[j]);
            I[j] = j;
            move16();
        }

        j = sub(len,1);
        FOR (k1=0; k1<j; k1++)
        {
            FOR (k2=add(k1,1); k2<len; k2++)
            {
                IF (sub(tmp1[k2],tmp1[k1])>0)
                {
                    tmpi = I[k2];
                    move16();
                    tmpf = tmp1[k2];
                    move16();
                    tmp1[k2] = tmp1[k1];
                    move16();
                    I[k2] = I[k1];
                    move16();
                    tmp1[k1] = tmpf;
                    move16();
                    I[k1] = tmpi;
                    move16();
                }
            }
        }

        /*using a factor of 1.37 to compensate for the ~ 2.5 ( or 2.73) dB diff between this scheme and EVS-UV */
        FOR (j=0; j<cnt; j++)
        {
            L16 = mult_r(tmp[I[j]], gain_fac); /* Q14 */
            L16 = mult_r(L16, 0x6EDA); /* Q13 */

            output[i*25+I[j]]= round_fx(L_shl(L_mult(L16,Gains[i]),2)); /* Q_exc */
        }
        FOR (; j<len; j++)
        {
            output[i*25+I[j]] = 0;
            move16();
        }
    }
}
