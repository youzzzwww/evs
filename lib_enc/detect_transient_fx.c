/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"    /* Function prototypes                    */
#include "stl.h"        /* required for wmc_tool */

/*--------------------------------------------------------------------------*/
/*  Function  hp_filter                                                     */
/*  ~~~~~~~~~~~~~~~~~~~~                                                    */
/*                                                                          */
/*  High pass filter                                                        */
/*--------------------------------------------------------------------------*/
/*  float      x      (i)    in Q_new input to filter                       */
/*  float      y      (o)    in Q_new +2 output of filter                   */
/*  float      *oldy  (i/o)  previous output of filter						*/
/*  float      *oldx  (i/o)  in Q_memx previous input of filter             */
/*  short      L      (i)    in Q_memx +2  length (32 or 48 kHz)            */
/*--------------------------------------------------------------------------*/

static void hp_filter_fx(
    const Word16 *x, /*Q_new */
    Word16 *y, /*Q_new */
    Word16 *oldy, /*Q_new */
    Word16 *oldx, /*Q_new */
    const Word16 L
)
{
    Word16 i;
    Word32 L_tmp;


    /*y[0] = 0.4931f * *oldy + 0.7466f*(x[0] - *oldx); */
    L_tmp = L_mult(sub(x[0],*oldx),24465);/*Q_new+16 */
    L_tmp = L_mac(L_tmp,*oldy,16158);/*Q_new+16 */
    y[0] = round_fx(L_tmp);/*Q_new */

    FOR (i = 1; i < L; i++)
    {
        /*y[i] = 0.4931f*y[i-1] + 0.7466f*(x[i] - x[i-1]); */
        L_tmp = L_mult(sub(x[i],x[i-1]),24465);/*Q_new+16 */
        L_tmp = L_mac(L_tmp,y[i-1],16158);/*Q_new+16 */
        y[i] = round_fx(L_tmp);           /*Q_new */
    }

    *oldx = x[L - 1];
    move16();/*Q_new */
    *oldy = y[L - 1];
    move16();/*Q_new */

}
/*--------------------------------------------------------------------------*/
/*  Function  detect_transient                                              */
/*  ~~~~~~~~~~~~~~~~~~~~~~~~~~                                              */
/*                                                                          */
/*  Detect if the signal is a transient                                     */
/*--------------------------------------------------------------------------*/
/*  float          in[]    (i)    input signal                        Q_new */
/*  Encoder_State *st      (i/o)  state of coder                            */
/*  short          L       (i)    length (32 or 48 kHz)                     */
/*--------------------------------------------------------------------------*/
/*  short       return     (o)   result of transient check                  */
/*--------------------------------------------------------------------------*/

Word16 detect_transient_fx(
    const Word16 *in_fx, /*Q_new */
    /*Encoder_State *st, */
    const Word16 L,
    const Word16 coder_type ,          /* i  : coder type               */
    Word16 Q_new,
    Encoder_State_fx *st_fx
)
{
    Word32 Energy,L_tmp;
    Word32 EnergyLT;
    Word16 i, blk;
    Word16 IsTransient;
    Word16 out_filt_fx[L_FRAME48k];
    Word16 position = 0;
    Word16 thr;
    Word32 L_tmp2;
    Word16 shift;
    Word32 Energy_fx, E_in_fx = 0, E_out_fx = 0, Energy_in_fx[5] = {0,0,0,0,0}; /* Energy_fx can be replaced by Energy */
    Word32 E_low_fx, E_high_fx;
    Word16 temp16, Thres_fx = 0;
    Word16 exp;

    shift = 0;
    move16();

    IsTransient = 0;
    move16();
    IF (sub(st_fx->last_extl_fx, st_fx->extl_fx) != 0)
    {
        st_fx->TransientHangOver_fx = 0;
        move16();
        st_fx->old_hpfilt_in_fx = 0;
        move16();
        st_fx->old_hpfilt_out_fx = 0;
        move16();
    }

    /* High pass filter */
    hp_filter_fx(in_fx, out_filt_fx, &(st_fx->old_hpfilt_in_fx), &(st_fx->old_hpfilt_out_fx), L);

    /* Long term energy */
    test();
    test();
    test();
    IF (sub(st_fx->last_extl_fx, st_fx->extl_fx) != 0 || (sub(st_fx->last_extl_fx, st_fx->extl_fx) == 0 && sub(st_fx->last_core_fx, st_fx->core_fx) != 0) || sub(st_fx->last_codec_mode, MODE2) == 0)
    {
        /*EnergyLT = EPSILON_FX; */
        EnergyLT = L_deposit_l(0);
        FOR (i = 0; i < L/4; i++)
        {
            /*EnergyLT += out_filt[i] * out_filt[i]; */
            EnergyLT = L_mac0(EnergyLT, out_filt_fx[i], out_filt_fx[i]);		/*2Q_new */
        }
    }
    ELSE
    {
        EnergyLT = L_add(st_fx->EnergyLT_fx, 0);									/*2Q_new */
    }
    IF (sub(L, L_FRAME8k) == 0)
    {
        Energy_in_fx[0] = st_fx->Energy_Old_fx;
        move32();
        /* Compute block energy */
        FOR ( blk = 0; blk < 4; blk++ )
        {
            Energy_fx = L_deposit_l(0);
            Energy_in_fx[blk + 1] = L_deposit_l(0);
            FOR ( i = 0; i < L/4; i++ )
            {
                temp16 = extract_l(L_shr(out_filt_fx[i + blk*(L/4)], 12));
                Energy_fx = L_add(Energy_fx, L_mult0(temp16, temp16));
                temp16 = shr(in_fx[i + blk*(L/4)], Q_new);
                Energy_in_fx[blk+1] = L_add(Energy_in_fx[blk+1], L_mult0(temp16, temp16));
                move32();
            }

            E_in_fx = L_add(E_in_fx, Energy_in_fx[blk + 1]);
            E_out_fx = L_add(E_out_fx, Energy_fx);

            Thres_fx = 2185;/*1 /15	 */
            move16();
            IF (L_sub(Mult_32_16(Energy_fx, 5461), EnergyLT) > 0)
            {
                IsTransient = 1;
                move16();
                position = blk;
                move16();
            }

            EnergyLT = L_add(Mult_32_16(EnergyLT, 24576), Mult_32_16(Energy_fx, 8192));
        }
    }
    ELSE
    {
        /* Compute block energy */
        FOR ( blk = 0; blk < 4; blk++ )
        {
            L_tmp = L_deposit_l(0);
            FOR ( i = 0; i < L/8; i++ )
            {
                /*Energy += out_filt_fx[i + blk*(L/4)] * out_filt_fx[i + blk*(L/4)]; */
                L_tmp = L_mac0(L_tmp, out_filt_fx[i + blk*(L/4)], out_filt_fx[i + blk*(L/4)]);  /*2Q_new */
            }
            L_tmp2 = L_deposit_l(0);
            FOR (; i < L/4; i++ )
            {
                /*Energy += out_filt_fx[i + blk*(L/4)] * out_filt_fx[i + blk*(L/4)]; */
                L_tmp2 = L_mac0(L_tmp2, out_filt_fx[i + blk*(L/4)], out_filt_fx[i + blk*(L/4)]);  /*2Q_new */
            }
            Energy = L_add(L_tmp, L_tmp2);
            shift = 0;
            if (Overflow != 0)
            {
                shift = 1;
                move16();
            }
            Overflow = 0;
            move16();

            Energy = L_add(L_shr(L_tmp, shift), L_shr(L_tmp2, shift));

            test();
            IF( sub(st_fx->extl_fx,SWB_BWE) == 0 || sub(st_fx->extl_fx,FB_BWE) == 0 )
            {
                /*Calculate shift to get to Q0*/
                test();
                test();
                IF((L_sub(Mult_32_16(Energy, shl(2427, shift)), EnergyLT) > 0) || (L_sub(Mult_32_16(Energy, shl(3277, shift)), EnergyLT) > 0 && sub(coder_type,INACTIVE) == 0))
                {
                    IsTransient = 1;
                    move16();
                    position = blk;
                    move16();
                }
            }
            ELSE
            {
                test();
                IF( L_sub(st_fx->total_brate_fx,HQ_16k40) <= 0&& sub(st_fx->bwidth_fx,SWB) == 0 )
                {
                    thr = 2427;
                    move16();
                }
                ELSE
                {
                    thr = 5461;
                    move16();
                }
                thr = shl(thr, shift);
                /*if(Energy > L_shr(Mult_32_16(EnergyLT,22624),shift_cnt))				//getting in Q0   32*16 = Q_inp1+Q_inp2+1-16 */
                IF(L_sub(Mult_32_16(Energy, thr),EnergyLT) > 0)
                /*if(Energy > 6.0f * EnergyLT)  */
                {
                    IsTransient = 1;
                    move16();
                    position = blk;
                    move16();
                }
            }
            /*EnergyLT = 0.75f*EnergyLT + 0.25f*Energy;				 */
            /*0.75f*EnergyLT in Q0                         //0.25f*Energy in Q0			 */
            EnergyLT = L_add(Mult_32_16(EnergyLT,24576),Mult_32_16(Energy, shl(8192, shift))); /*2Q_new */
        }
    }
    st_fx->EnergyLT_fx = EnergyLT;
    move32();

    test();
    test();
    test();
    test();
    test();
    if( ( sub(st_fx->last_extl_fx,SWB_BWE) != 0 && sub(st_fx->last_extl_fx,SWB_TBE) != 0 && sub(st_fx->extl_fx,SWB_BWE) == 0 ) ||
            ( sub(st_fx->last_extl_fx,FB_BWE) != 0 && sub(st_fx->last_extl_fx,FB_TBE) != 0 && sub(st_fx->extl_fx,FB_BWE) == 0 ) )
    {
        IsTransient = 0;
        move16();
    }

    test();
    IF ( IsTransient && L == L_FRAME8k )
    {
        blk = 0;
        move16();
        E_low_fx = L_deposit_l(0);
        FOR (i=0; i<position+1; i++)
        {
            /*blk++;	 */
            blk = add(blk, 1);
            E_low_fx = L_add(E_low_fx, Energy_in_fx[i]);
        }

        exp = norm_s(blk);
        temp16 = div_s(16384, shl(blk, exp));/* 15 + 14 - exp; */
        exp = 15 + 14 - exp;
        temp16 = shl(temp16, sub(15, exp));
        E_low_fx = Mult_32_16(E_low_fx, temp16);

        blk = 0;
        move16();
        E_high_fx = L_deposit_l(0);
        FOR (i=position+1; i<5; i++)
        {
            /*blk++; */
            blk = add(blk, 1);
            E_high_fx = L_add(E_high_fx, Energy_in_fx[i]);
        }

        exp = norm_s(blk);
        temp16 = div_s(16384, shl(blk, exp));/* 15 + 14 - exp; */
        exp = 15 + 14 - exp;
        temp16 = shl(temp16, 15 - exp);
        E_high_fx = Mult_32_16(E_high_fx, temp16);

        test();
        test();
        IF (L_sub(L_shr(E_high_fx, 1), E_low_fx) < 0 && L_sub(E_high_fx, Mult_32_16(E_low_fx, 22938)) > 0 && L_sub(Mult_32_16(E_in_fx, Thres_fx), E_out_fx) > 0)
        {
            IsTransient = 0;
            move16();
        }
    }
    IF ( L_sub(st_fx->core_brate_fx,ACELP_24k40) == 0 )
    {
        test();
        IF ( sub(st_fx->last_core_fx,HQ_CORE) != 0 || L_sub(st_fx->last_core_brate_fx,ACELP_24k40) != 0 )
        {
            st_fx->TransientHangOver_fx = 0;
            move16();
            IsTransient = 0;
            move16();
        }
        ELSE
        {
            IF ( IsTransient )
            {
                IF ( sub(position,3) == 0 )
                {
                    /* Set Hangover */
                    st_fx->TransientHangOver_fx = 1;
                    move16();
                }
            }
            ELSE
            {
                IF ( st_fx->TransientHangOver_fx )
                {
                    st_fx->TransientHangOver_fx = 0;
                    move16();
                    IsTransient = 1;
                    move16();
                }
            }
        }
    }
    ELSE
    {
        IF ( IsTransient )
        {
            st_fx->TransientHangOver_fx = 1;
            move16();
        }
        ELSE
        {
            IF( st_fx->TransientHangOver_fx )
            {
                st_fx->TransientHangOver_fx = 0;
                move16();
                IsTransient = 1;
                move16();
            }
        }
    }
    st_fx->Energy_Old_fx = Energy_in_fx[4];
    move32();
    return IsTransient;
}
