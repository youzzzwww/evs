/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "rom_com_fx.h"   /* Static table prototypes                */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
 * Local constants
 *-------------------------------------------------------------------*/

#define ATT_LENGHT            64
#define ATT_SEG_LEN           (L_FRAME/ATT_LENGHT)
#define INV_ATT_SEG_LEN       (1.0f/ATT_SEG_LEN)
#define INV_L_FRAME           (1.0f/L_FRAME)

/*==========================================================================*/
/* FUNCTION : void pre_echo_att_fx();								        */
/*--------------------------------------------------------------------------*/
/* PURPOSE  :  Attenuation of the pre-echo when encoder specifies an attack */
/*--------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												        */
/* _ (Word16) L_frame_fx         : length of the frame			Q0		    */
/* _ (Word16) gsc_attack_flag_fx : LP filter coefficient		Q0		    */
/* _ (Word16) core_fx            : core codec used 				Q0		    */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													    */
/* _ None                                                                   */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											    */
/* _ (Word16*) exc_fx			   : adapt. excitation exc       Q_exc  	*/
/* _ (Word32*) Last_frame_ener_fx  : Energy of the last frame    Q1         */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													    */
/* _ None																    */
/*==========================================================================*/
void pre_echo_att_fx(
    Word32 *Last_frame_ener_fx,     /* i/o: Energy of the last frame         2*Q_new+1*/
    Word16 *exc_fx,                 /* i/o: Excitation of the current frame  Q_new*/
    const Word16 gsc_attack_flag_fx       /* i  : flag signalling attack encoded by AC mode (GSC) */
    ,const Word16 Q_new
    ,const Word16 last_coder_type_fx       /* i  : Last coding mode */
)
{
    Word32 etmp_fx;
    Word32 finc_fx[ATT_LENGHT] = {0};
    Word16 ratio_fx;
    Word16 attack_pos_fx, i;
    Word32 L_tmp, L_tmp1;
    Word16 tmp, n1, n2, exp, frac1, frac2;
    Word32 etmp1_fx;
    test();
    IF ( sub(gsc_attack_flag_fx,1) == 0  && sub(last_coder_type_fx, AUDIO) == 0) /*gsc_attack_flag_fx does not get set for all the test cases */
    {
        /*-------------------------------------------------------------------------*
         * Find where the onset (attack) occurs by computing the energy per section
         * The inverse weighting aims to favor the first maxima in case of
         * gradual onset
         *-------------------------------------------------------------------------*/
        FOR(i = 0; i < ATT_LENGHT; i++)
        {
            L_tmp = sum2_fx(&exc_fx[shl(i,2)], ATT_SEG_LEN ); /*2*Q_new+1, //ATT_SEG_LEN=(L_FRAME/ATT_LENGHT)=4(=shl(x,2))*/
            tmp = div_s(sub(ATT_LENGHT,i),ATT_LENGHT);  /*Q15 */
            L_tmp = Mult_32_16(L_tmp, tmp);             /*2*Q_new+1 */
            finc_fx[i] = L_tmp;
            move32();    /*2*Q_new+1 */
        }

        attack_pos_fx = maximum_32_fx(finc_fx, ATT_LENGHT, &etmp_fx);

        /* Scaled the maximum energy and allowed 6 dB increase*/
        etmp_fx = L_shr(etmp_fx,add(2+1-4, shl(Q_new,1)));/*2*Q_new+1 //INV_ATT_SEG_LEN=1/4(=shr(x,2)) -> Q4 */
        etmp1_fx = etmp_fx;
        move32();
        *Last_frame_ener_fx = L_shl(*Last_frame_ener_fx,2);
        move32(); /*2*Q_new+1 */

        /* If the maximum normalized energy > last frame energy + 6dB */
        IF( L_sub(etmp_fx,*Last_frame_ener_fx) > 0 )
        {
            /* Find the average energy before the attack */
            L_tmp = sum32_fx( finc_fx, attack_pos_fx);              /*Q1 */
            L_tmp1 = L_shr(L_mult(attack_pos_fx,attack_pos_fx),1);  /*Q0 */
            tmp = round_fx(Isqrt(L_tmp1));                          /*Q15 */
            L_tmp = L_shr(L_tmp,2);                                 /*Q1 ; ATT_SEG_LEN=4 */
            etmp_fx = Mult_32_16(L_tmp,tmp);                        /*Q1 */

            /* Find the correction factor and apply it before the attack */
            /*   ratio = (float)sqrt(*Last_frame_ener/etmp);*/
            /*         = isqrt(etmp/(*Last_frame_ener))			*/
            etmp_fx = L_max(etmp_fx,1);
            *Last_frame_ener_fx = L_max(*Last_frame_ener_fx,1);
            n1 = norm_l(etmp_fx);
            n2 = norm_l(*Last_frame_ener_fx);

            n1 = sub(n1,1);
            exp = sub(n1,n2);

            frac1 = round_fx(L_shl(etmp_fx,n1));
            frac2 = round_fx(L_shl(*Last_frame_ener_fx,n2));

            L_tmp = L_mult0(128, div_s(frac1, frac2)); /* s = gain_out / gain_in */
            L_tmp = L_shr(L_tmp, exp);   /* add exponent */

            L_tmp = Isqrt(L_tmp);
            ratio_fx = round_fx(L_shl(L_tmp, 9));

            FOR(i = 0; i < attack_pos_fx*ATT_SEG_LEN; i++)
            {
                /*exc_fx[i] *= ratio_fx;*/
                exc_fx[i] = round_fx(L_shl(L_mac(-8192, exc_fx[i], ratio_fx), 2));
            }
        }
        *Last_frame_ener_fx = etmp1_fx;
        move32();
    }
    ELSE
    {
        /*-------------------------------------------------------*
         * In normal cases, just compute the energy of the frame
         *-------------------------------------------------------*/

        etmp_fx = sum2_fx( exc_fx, L_FRAME ); /*2*Q_new+1 */

        etmp_fx = L_shr(etmp_fx,add(8+1-4, shl(Q_new,1))); /*2*Q_new+1 //INV_L_FRAME = 1/256 -> Q4*/
        *Last_frame_ener_fx = etmp_fx;
        move32(); /*2*Q_new+1*/
    }

    return;
}
