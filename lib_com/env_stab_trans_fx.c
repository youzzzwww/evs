/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"       /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "math_op.h"       /* WMOPS macros                           */
#include "stl.h"           /* required by wmc_tool */

/*--------------------------------------------------------------------------*
 * env_stab_transient_detect()
 *
 * Transient detector for envelope stability measure
 *--------------------------------------------------------------------------*/

void env_stab_transient_detect_fx(
    const Word16 is_transient,         /* i:   Transient flag												*/
    const Word16 length,               /* i  : Length of spectrum (32 or 48 kHz)							*/
    const Word16 norm[],               /* i  : quantization indices for norms								*/
    Word16 *no_att_hangover,     /* i/o: Frame counter for attenuation hangover			 (Q0)		*/
    Word32 *L_energy_lt,         /* i/o: Long-term energy measure for transient detection  (Q13)      */
    const Word16 HQ_mode,              /* i  : HQ coding mode												*/
    const Word16 bin_th,               /* i  : HVQ cross-over frequency bin									*/
    const Word32 *L_coeff,             /* i  : Coded spectral coefficients                            		*/
    const Word16 Qcoeff                /* i  : Q of coded spectral coefficients                         	*/
)
{
    Word16 i, blk, norm_ind, sqrt_exp, bin_th_1, temp, sh;
    Word32 L_e_frame, L_temp, L_d_max;
    Word32 L_energy_lt_local;
    Word32 L_E_sub[4];
    Word32 L_delta_e_sub;

    L_energy_lt_local = *L_energy_lt;
    move32();

    L_d_max = L_deposit_l(0);
    L_e_frame = L_deposit_l(0);
    temp = 32;
    move16();

    IF( sub(HQ_mode,HQ_HVQ) == 0 )
    {
        FOR (i = 0; i < bin_th; i++)                                                            /* find adaptive shift */
        {
            temp = s_min(temp,norm_l(L_coeff[i]));
        }
        sh = sub(temp,2);                                                                       /* scale such that 2 msbs are not used, the resulting adaptive Qcoeff will be: Qcoeff+sh-16 */
        FOR (i = 0; i < bin_th; i++)                                                            /* Maximum number of loop runs 320 */
        {
            temp = extract_h(L_shl(L_coeff[i],sh));
            L_e_frame = L_mac(L_e_frame,temp,temp);                                             /* Q(2*(Qcoeff+sh-16)+1)=Q(2*(Qcoeff+sh)-31 */
        }

        bin_th_1 = INV_HVQ_THRES_BIN_24k;
        move16();
        if (sub(bin_th, HVQ_THRES_BIN_32k) == 0)
        {
            bin_th_1 = INV_HVQ_THRES_BIN_32k;
            move16();
        }
        L_temp = Mult_32_16(L_e_frame,bin_th_1);                                                  /* Q(2*(Qcoeff-16+sh)+1+21-15) -> Q(2*(Qcoeff+sh)-25)  */
        L_e_frame = Sqrt_l(L_temp,&sqrt_exp);
        L_e_frame = L_shr(L_e_frame, add(sub(add(sh,Qcoeff),10),shr(sqrt_exp,1)));                /* Adjust by (Qcoeff+sh-10) to fixed Q13: Qcoeff+sh+(-25+31)/2 - (Qcoeff+sh-10) -> Q13 */

        IF ( L_sub(L_e_frame, ENERGY_TH_FX) > 0 )
        {
            L_energy_lt_local = Mult_32_16(*L_energy_lt, ENERGY_LT_BETA_FX);
            L_temp = Mult_32_16(L_e_frame, ENERGY_LT_BETA_1_FX);
            *L_energy_lt = L_add(L_energy_lt_local,L_temp);
            move32();
        }

        IF (*no_att_hangover > 0)
        {
            (*no_att_hangover) = sub((*no_att_hangover), 1);
            move16();
        }
    }
    ELSE
    {
        L_e_frame = L_deposit_l(0);

        test();
        IF (is_transient && sub(length,L_FRAME32k) == 0)
        {
            /* Measure subframe energies */
            FOR (blk = 0; blk < NUM_SUBFRAMES; blk++)
            {
                L_E_sub[blk] = L_deposit_l(0);                                                         /* Q9 */

                FOR (i=0; i<BANDS_PER_SUBFRAMES; i++)                                           /* 9 times -> < 2^4 */
                {
                    norm_ind = subf_norm_groups_fx[blk][i];
                    move16();
                    L_E_sub[blk] = L_add(L_E_sub[blk],L_shr(dicn_fx[norm[norm_ind]],4));
                    move32(); ;             /* Q10  */
                }

                L_E_sub[blk] = Mult_32_16(L_E_sub[blk], INV_BANDS_PER_SUBFRAMES);
                move32();                       /* Q(10+17-15) -> Q12 */

                L_e_frame = L_add(L_e_frame,L_E_sub[blk]);                                      /* Q12 */
            }

            /* Test for transient */
            /*            if (e_frame > ENERGY_TH * NUM_SUBFRAMES) */
            IF (L_sub(L_e_frame, ENERGY_TH_NUM_SUBFRAMES) > 0)
            {
                FOR (blk = 0; blk < NUM_SUBFRAMES-1; blk++)
                {
                    L_delta_e_sub = L_sub(L_E_sub[blk+1],L_E_sub[blk]);                         /* Q12 */
                    if (L_sub(L_delta_e_sub,L_d_max)>0)
                    {
                        L_d_max = L_add(L_delta_e_sub,0);                                      /* L_d_max is NOT normalized with *energy_lt */
                    }
                }
            }
        }
        ELSE
        {
            /* Update long-term energy measure */
            L_e_frame = L_deposit_l(0);                                                         /* Q9 */
            FOR (i = 0; i < SFM_N_ENV_STAB; i++)                                                /* 27 times -> < 2^5 */
            {
                L_e_frame = L_add(L_e_frame,L_shr(dicn_fx[norm[i]],5));
                /* Q9  */
            }

            L_e_frame = Mult_32_16(L_e_frame, INV_SFM_N_ENV_STAB);                              /* Q(9+19-15) -> Q13 */

            IF ( L_sub(L_e_frame, ENERGY_TH_FX) > 0 )
            {
                L_energy_lt_local = Mult_32_16(*L_energy_lt, ENERGY_LT_BETA_FX);
                L_temp = Mult_32_16(L_e_frame, ENERGY_LT_BETA_1_FX);
                *L_energy_lt = L_add(L_energy_lt_local,L_temp);
                move32();
            }
        }

        /* Add hang-over for conservative application of stability dependent attenuation */
        /*                              -> Note: L_d_max not normalized with *energy_lt */
        /*                                 Hence, we compare L_d_max/DELTA_TH with *energy_lt */
        IF (L_sub(Mult_32_16(L_d_max, INV_DELTA_TH),L_energy_lt_local) > 0)         /* Q13 = Q(12 + 16 -15) */
        {
            *no_att_hangover = ATT_LIM_HANGOVER;
            move16();
        }
        ELSE if (*no_att_hangover > 0)
        {
            *no_att_hangover = sub(*no_att_hangover,1);
            move16();
        }
    }

    return;
}

