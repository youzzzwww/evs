/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define ALPHA0_FX  13107
#define BETA0_FX  (32768-ALPHA0_FX)

/*========================================================================*/
/* FUNCTION : Inac_swtch_ematch_fx()									  */
/*------------------------------------------------------------------------*/
/* PURPOSE : Apply energy matching when swithcing to INACTIVE frame coded */
/*			 by the GSC technology										  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16) coder_type  : Coding mode									  */
/* _ (Word16) L_frame	  : Frame lenght						          */
/* _ (Word32) core_brate  : core bitrate   						          */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/* _ (Word16[]) exc2	         : CELP/GSC excitation buffer     Q_exc   */
/* _ (Word16[]) lt_ener_per_band : Long term energy per band      Q12     */
/* _ (Word16*) Q_exc             : input and output format of exc2        */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/
void Inac_swtch_ematch_fx(
    Word16 exc2[],               /* i/o: CELP/GSC excitation buffer       Q_exc*/
    Word16 dct_exc_tmp[],        /* i  : GSC excitation in DCT domain          */
    Word16 lt_ener_per_band[],   /* i/o: Long term energy per band        Q12  */
    const Word16 coder_type,           /* i  : Coding mode                           */
    const Word16 L_frame,              /* i  : Frame lenght                          */
    const Word32 core_brate,           /* i  : Core bit rate					     */
    const Word16 Q_exc				   /* i  : input and output format of exc2       */
)
{
    Word16 Ener_per_bd[MBANDS_GN];
    Word16 ftmp;
    Word16 *pt_exc;
    Word16 j, i;

    Word16 exp,frac;
    Word32 L_tmp;

    /*--------------------------------------------------------------------------
     * average energy per band
     *--------------------------------------------------------------------------*/

    test();
    test();
    test();
    IF(sub(coder_type,AUDIO) == 0)
    {
        Ener_per_band_comp_fx( dct_exc_tmp, Ener_per_bd, Q_exc, MBANDS_GN, 1);

        /* reset long-term energy per band */
        FOR(i = 0; i < MBANDS_GN; i++)
        {
            lt_ener_per_band[i] = Ener_per_bd[i];
            move16();
        }

    }
    ELSE IF( sub(coder_type,VOICED) == 0 || sub(coder_type,GENERIC) == 0 || sub(coder_type,TRANSITION) == 0 )
    {
        /* Find spectrum and energy per band for GC and VC frames */
        edct_16fx( exc2, dct_exc_tmp, L_frame, 5 );

        Ener_per_band_comp_fx( dct_exc_tmp, Ener_per_bd, Q_exc, MBANDS_GN, 1);

        /* reset long-term energy per band */
        FOR(i = 0; i < MBANDS_GN; i++)
        {
            lt_ener_per_band[i] = Ener_per_bd[i];
            move16();
        }
    }
    ELSE IF( sub(coder_type,INACTIVE) == 0 && L_sub(core_brate,ACELP_24k40) <= 0)
    {
        /* Find spectrum and energy per band for inactive frames */
        edct_16fx( exc2, dct_exc_tmp, L_frame, 5 );
        Ener_per_band_comp_fx( dct_exc_tmp, Ener_per_bd, Q_exc, MBANDS_GN, 1 );

        /* More agressive smoothing in the first 50 frames */
        pt_exc = dct_exc_tmp;
        move16();
        FOR(i = 0; i < MBANDS_GN; i++)
        {
            /* Compute smoothing gain to apply with gain limitation */
            L_tmp = L_mult(ALPHA0_FX,lt_ener_per_band[i]); /*Q(15+12+1)=Q(28) */
            L_tmp = L_mac(L_tmp,BETA0_FX,Ener_per_bd[i]); /*Q28 */
            lt_ener_per_band[i] = round_fx(L_tmp); /*Q12 */

            ftmp = sub(lt_ener_per_band[i],Ener_per_bd[i]); /*Q12 */

            /* ftmp = (float)pow(10, ftmp);= pow(2,3.321928*ftmp);*/

            L_tmp = L_mult(27213,ftmp); /*Q(13+12+1)=Q26 ; 27213=3.321928 in Q13 */
            L_tmp = L_shr(L_tmp, 10); /* From Q26 to Q16 */
            frac = L_Extract_lc(L_tmp, &exp); /* Extract exponent of ftmp */
            ftmp = extract_l(Pow2(14, frac));/* Put 14 as exponent so that */
            /* output of Pow2() will be: */
            /* 16384 < Pow2() <= 32767 */

            exp = sub(exp,14);
            IF( sub(i,2) < 0 )
            {
                FOR (j = 0; j < 8; j ++)
                {
                    L_tmp = L_mult(*pt_exc,ftmp);  /* Q_exc*Q0 -> Q(Q_exc+1) */
                    L_tmp = L_shl(L_tmp, add(exp,15));  /* Q(Q_exc+1) -> Q(16+Q_exc)*/
                    *pt_exc = round_fx(L_tmp);
                    pt_exc++;
                }
            }
            ELSE
            {
                FOR (j = 0; j < 16; j ++)
                {
                    L_tmp = L_mult(*pt_exc,ftmp);       /* Q_exc*Q0 -> Q(Q_exc+1) */
                    L_tmp = L_shl(L_tmp, add(exp,15));  /* Q(Q_exc+1) -> Q(16+Q_exc)*/
                    *pt_exc = round_fx(L_tmp);          /*Q_exc*/
                    pt_exc++;
                }
            }
        }

        /* Going back to time */
        edct_16fx( dct_exc_tmp, exc2, L_frame, 5 );
    }

    return;
}
