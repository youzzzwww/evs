/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"    /* Common constants                       */
#include "rom_com_fx.h" /* Static table prototypes                */
#include "prot_fx.h"    /* Function prototypes                    */
#include "stl.h"        /* required for wmc_tool */

void bitalloc_fx (
    Word16 *y,                /* i  : reordered norm of sub-vectors              Q0 */
    Word16 *idx,              /* i  : reordered sub-vector indices               Q0 */
    Word16 sum,               /* i  : number of available bits                   Q0 */
    Word16 N,                 /* i  : number of norms                            Q0 */
    Word16 K,                 /* i  : maximum number of bits per dimension       Q0 */
    Word16 *r,                /* o  : bit-allacation vector                      Q0 */
    const Word16 *sfmsize,          /* i  : band length                                Q0 */
    const Word16 hqswb_clas         /* i  : signal classification flag                 Q0 */
)
{
    Word16 i, j, k, n, m, v, im;
    Word16 diff, temp;
    Word16 fac;
    Word16 ii;
    Word16 SFM_thr = SFM_G1G2;
    move16();

    N = sub(N, 1);

    if ( sub(hqswb_clas, HQ_HARMONIC) == 0 )
    {
        SFM_thr = 22;
        move16();
    }

    fac = 3;
    move16();
    K = sub(K,2);
    im = 1;
    move16();
    diff = sum;
    move16();
    n = shr(sum,3);
    FOR ( i=0; i<n; i++ )
    {
        k = 0;
        move16();
        temp = y[0];
        move16();
        FOR ( m=1; m<im; m++)
        {
            v = sub( temp, y[m] );
            temp = s_max(temp, y[m]);
            if ( v < 0 )
            {
                k = m;
                move16();
            }
        }

        IF ( sub(temp, y[m]) < 0 )
        {
            k = m;
            move16();
            if ( sub(im, N) < 0 )
            {
                im = add(im, 1);
            }
        }

        j = idx[k];
        move16();

        test();
        IF ( sub(sum,sfmsize[j]) >= 0 && sub(r[j],K) < 0 )
        {
            y[k] = sub(y[k], fac);
            move16();
            r[j] = add(r[j], 1);
            move16();

            if ( sub(r[j], K) >= 0 )
            {
                y[k] = -32768;
                move16();
            }
            sum = sub(sum,sfmsize[j]);
        }
        ELSE
        {
            y[k] = -32768;
            move16();
            k = add(k, 1);
            test();
            if ( sub(k, im) == 0 && sub(im, N) < 0 )
            {
                im = add(im, 1);
            }
        }

        test();
        IF ( (sub(sum, WID_G1)<0) || (sub(diff, sum)==0) )
        {
            BREAK;
        }

        diff = sum;
        move16();
        v = sub(N, 1);

        IF ( sub(k, v) > 0 )
        {
            FOR ( ii=0; ii<=N; ii++ )
            {
                IF ( sub(y[ii], -32768) > 0 )
                {
                    if ( sub(ii, N) < 0 )
                    {
                        im = add(ii, 1);
                    }
                    BREAK;
                }
            }
        }
    }


    IF ( sub(sum, WID_G2) >= 0 )
    {
        FOR (i=0; i<=N; i++)
        {
            j = idx[i];
            move16();
            test();
            test();
            IF ( sub(j, SFM_G1) >= 0 && sub(j, SFM_thr) < 0 && r[j] == 0 )
            {
                r[j] = 1;
                move16();
                sum = sub(sum, WID_G2);
                IF (sub(sum, WID_G2) < 0)
                {
                    BREAK;
                }
            }
        }
    }

    IF ( sub(sum, WID_G2) >= 0 )
    {
        FOR (i=0; i<=N; i++)
        {
            j = idx[i];
            move16();
            test();
            test();
            IF ( sub(j,SFM_G1) >= 0 && sub(j, SFM_thr) < 0 && sub(r[j], 1) == 0 )
            {
                r[j] = 2;
                move16();
                sum = sub(sum, WID_G2);
                IF ( sub(sum, WID_G2) < 0 )
                {
                    BREAK;
                }
            }
        }
    }

    IF ( sub(sum, WID_G1) >= 0 )
    {
        FOR (i=0; i<=N; i++)
        {
            j = idx[i];
            move16();
            test();
            IF ( sub(j, SFM_G1) < 0 && r[j] == 0 )
            {
                r[j] = 1;
                move16();
                sum = sub(sum, WID_G1);
                IF ( sub(sum, WID_G1) < 0 )
                {
                    BREAK;
                }
            }
        }
    }

    IF ( sub(sum, WID_G1) >= 0 )
    {
        FOR (i=0; i<=N; i++)
        {
            j = idx[i];
            move16();
            test();
            IF ( sub(j, SFM_G1) < 0 && sub(r[j], 1) == 0 )
            {
                r[j] = 2;
                move16();
                sum = sub(sum, WID_G1);
                IF ( sub(sum, WID_G1) < 0 )
                {
                    BREAK;
                }
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * BitAllocF()
 *
 * Fractional bit allocation
 *-------------------------------------------------------------------*/

Word16 BitAllocF_fx (
    Word16 *y,                /* i  : norm of sub-vectors                      :Q0  */
    Word32  bit_rate,         /* i  : bitrate                                  :Q0  */
    Word16 B,                 /* i  : number of available bits                 :Q0  */
    Word16 N,                 /* i  : number of sub-vectors                    :Q0  */
    Word16 *R,                /* o  : bit-allocation indicator                 :Q0  */
    Word16 *Rsubband_fx       /* o  : sub-band bit-allocation vector           :Q3  */
    ,const Word16 hqswb_clas, /* i  : hq swb class                             :Q0  */
    const Word16 num_env_bands/* i  : Number sub bands to be encoded for HF GNERIC :Q0  */
)
{
    Word16 fac;
    Word16 i, n, Nmin, Bits, bs, low_rate = 0;

    Word16 m_fx;
    Word32 t_fx, B_fx;
    Word32 L_tmp1, L_tmp2, L_tmp3;
    Word16 tmp, exp1, exp2;
    Word32 Rsubband_w32_fx[NB_SFM];                                             /* Q15  */
    Word16 B_w16_fx;

    set32_fx( Rsubband_w32_fx, 0, NB_SFM);

    fac = 3;
    move16();

    IF (L_sub(bit_rate, 32000) < 0)
    {
        bs = 1;
        move16();
    }
    ELSE IF (L_sub(bit_rate, 48000) < 0)
    {
        bs = 2;
        move16();
    }
    ELSE IF (L_sub(bit_rate, 96000)< 0)
    {
        bs = 3;
        move16();
    }
    ELSE
    {
        bs = 4;
        move16();
    }
    bs = add(bs,1);

    if (L_sub(bit_rate,32000)<=0)
    {
        low_rate = 1;
        move16();
    }

    Nmin = N;
    move16();
    if ( sub(Nmin,SFM_N) > 0)
    {
        Nmin = SFM_N;
        move16();
    }

    /* Initial bits distribution */
    test();
    IF (sub(hqswb_clas , HQ_GEN_SWB) == 0 || sub(hqswb_clas , HQ_GEN_FB) == 0)
    {
        /* Initial bits distribution */
        L_tmp1 = 0;
        move16();
        m_fx = 0;
        move16();
        FOR ( i = 0; i < num_env_bands ; i++)
        {
            L_tmp1 = L_mac0(L_tmp1, Nb[i], y[i]);
        }
        L_tmp1 = L_msu0(L_tmp1, fac, B);

        t_fx = L_deposit_l(0);
        n = 0;
        move16();
        tmp = add(band_end[num_env_bands-1], shl(band_end[num_env_bands-1], 1));
        exp1 = norm_s(tmp);
        tmp = div_s(16384, shl(tmp, exp1));/*15 + 14 - exp1*/
        exp2 = norm_s(tmp);
        tmp = shl(tmp, exp2);
        exp1 = add(29, sub(exp2, exp1));

        FOR ( i = 0; i < N; i++)
        {
            L_tmp2 = L_sub(L_mult0(y[i], band_end[num_env_bands-1]), L_tmp1);
            Rsubband_w32_fx[i] = L_mult0(extract_l(L_tmp2), Nb[i]);
            move32();/*Q0*/
            IF (Rsubband_w32_fx[i] > 0)
            {
                n = add(n,Nb[i]);
                Rsubband_w32_fx[i] = Mult_32_16(Rsubband_w32_fx[i], tmp);
                move32();/*exp1 - 15*/
                Rsubband_w32_fx[i] = L_shl(Rsubband_w32_fx[i], sub(30, exp1));/*Q15*/

                t_fx = L_add(t_fx, Rsubband_w32_fx[i]);/*Q0*/
            }
            ELSE
            {
                Rsubband_w32_fx[i] = L_deposit_l(0);
            }
        }
    }
    ELSE
    {
        /* Initial bits distribution */
        L_tmp1 = 0;
        move16();
        m_fx = 0;
        move16();
        FOR ( i = 0; i < N ; i++)
        {
            L_tmp1 = L_mac0(L_tmp1, Nb[i], y[i]);
        }
        L_tmp1 = L_msu0(L_tmp1, fac, B);


        t_fx = L_deposit_l(0);
        n = 0;
        move16();
        tmp = add(band_end[N-1], shl(band_end[N-1], 1));
        exp1 = norm_s(tmp);
        tmp = div_s(16384, shl(tmp, exp1));/*15 + 14 - exp1*/
        exp2 = norm_s(tmp);
        tmp = shl(tmp, exp2);
        exp1 = add(29, sub(exp2, exp1));
        FOR ( i = 0; i < N; i++)
        {
            L_tmp2 = L_sub(L_mult0(y[i], band_end[N-1]), L_tmp1);
            Rsubband_w32_fx[i] = L_mult0(extract_l(L_tmp2), Nb[i]);
            move32();/*Q0*/
            IF (Rsubband_w32_fx[i] > 0)
            {
                n = add(n,Nb[i]);
                L_tmp3 = Mult_32_16(Rsubband_w32_fx[i], tmp);  /*exp1 - 15*/
                Rsubband_w32_fx[i] = L_shl(L_tmp3, sub(30, exp1)); /*Q15*/ move32();

                t_fx = L_add(t_fx, Rsubband_w32_fx[i]);/*Q0*/
            }
            ELSE
            {
                Rsubband_w32_fx[i] = L_deposit_l(0);
            }
        }
    }

    /* Distribute the remaining bits to subbands with non-zero bits */
    B_fx = L_shl(B, 15);
    WHILE (L_sub(L_shr(L_add(t_fx, 16384), 15) , B) != 0)
    {
        L_tmp1 = L_sub(t_fx, B_fx);
        exp1 = sub(norm_l(L_tmp1), 1);
        exp2 = norm_s(n);
        tmp = div_s(extract_h(L_shl(L_tmp1, exp1)), shl(n, exp2));/*15 + 15 + exp1 - 16 - exp2*/
        m_fx = shl(tmp, sub(exp2, exp1));/*Q14*/

        t_fx = L_deposit_l(0);
        n = 0;
        move16();
        FOR ( i = 0; i < N; i++)
        {
            IF (Rsubband_w32_fx[i] > 0)
            {
                Rsubband_w32_fx[i] = L_msu(Rsubband_w32_fx[i], m_fx, Nb[i]);
                move32();

                IF (Rsubband_w32_fx[i] > 0)
                {
                    n = add(n,Nb[i]);

                    t_fx = L_add(t_fx, Rsubband_w32_fx[i]);
                }
                ELSE
                {
                    Rsubband_w32_fx[i] = L_deposit_l(0);
                }
            }
        }
    }
    Bits = B;
    move16();

    /* Impose bit-constraints to subbands with less than minimum bits*/
    t_fx = L_deposit_l(0);
    n = 0;
    move16();
    FOR ( i = 0; i < N; i++)
    {
        IF (Rsubband_w32_fx[i] > 0)
        {
            test();
            IF ((L_sub(Rsubband_w32_fx[i] , L_shl(add(bs, LNb[i]), 15)) <0) && (sub(low_rate,1) == 0))
            {
                Rsubband_w32_fx[i] = L_deposit_l(0);
            }
            ELSE IF ( L_sub(Rsubband_w32_fx[i] , L_shl(Nb[i], 15)) <=0)
            {
                B = sub(B,Nb[i]);
                Rsubband_w32_fx[i] = L_shl(Nb[i], 15);
                move32();
            }
            ELSE
            {
                n = add(n,Nb[i]);
                t_fx = L_add(t_fx, Rsubband_w32_fx[i]);
            }
        }
    }

    /* Distribute the remaining bits to subbands with more than 1-bit per sample */
    WHILE (L_sub(L_shr(L_add(t_fx, 16384), 15) ,B) != 0)
    {
        L_tmp1 = L_sub(t_fx, L_shl(B, 15));
        L_tmp2 = L_abs(L_tmp1);

        if (n>0)
        {
            exp1 = sub(norm_l(L_tmp2), 1);
            exp2 = norm_s(n);
            tmp = div_s(extract_h(L_shl(L_tmp2, exp1)), shl(n, exp2));/*15 + 15 + exp1 - 16 - exp2*/
            m_fx = shl(tmp, sub(exp2, exp1));/*Q14*/
            if (L_tmp1 < 0)
            {
                m_fx = negate(m_fx);
            }

            t_fx = L_deposit_l(0);
            n = 0;
            move16();
            FOR( i = 0; i < N; i++)
            {
                IF (L_sub(Rsubband_w32_fx[i] , L_shl(Nb[i], 15)) > 0)
                {
                    Rsubband_w32_fx[i] = L_msu(Rsubband_w32_fx[i], m_fx, Nb[i]);
                    move32();
                    IF (L_sub(Rsubband_w32_fx[i] ,L_shl(Nb[i], 15)) > 0)
                    {
                        n   = add(n,Nb[i]);

                        t_fx = L_add(t_fx, Rsubband_w32_fx[i]);
                    }
                    ELSE
                    {
                        B = sub(B,Nb[i]);

                        Rsubband_w32_fx[i] = L_shl(Nb[i], 15);
                        move32();
                    }
                }
            }
        }
        /*In case no subband has enough bits more than 1-bit per sample, take bits off the higher subbands */
        IF (t_fx == 0)
        {
            FOR ( i = N-1; i >= 0; i--)
            {
                IF (Rsubband_w32_fx[i] > 0)
                {
                    B = add( B, Nb[i] );
                    Rsubband_w32_fx[i] = L_deposit_l(0);
                    IF ( B >= 0)
                    {
                        BREAK;
                    }
                }
            }
            BREAK;
        }
    }

    /* fine redistribution of over-allocated or under-allocated bits */
    tmp = 0;
    move16();
    FOR ( i = 0; i < N; i++)
    {
        Rsubband_fx[i] = extract_l(L_shr(Rsubband_w32_fx[i], 12));
        tmp = add(tmp, Rsubband_fx[i]);
    }

    B = Bits;
    B_w16_fx = shl(B, 3);
    IF (sub(tmp ,B_w16_fx)>0)
    {
        tmp = sub(tmp, B_w16_fx);
        FOR ( i = 0; i < N; i++)
        {
            IF (sub(Rsubband_fx[i], add(shl(Nb[i], 3), tmp)) >= 0)
            {
                Rsubband_fx[i] = sub(Rsubband_fx[i], tmp);
                move16();
                BREAK;
            }
        }
    }
    ELSE
    {
        tmp = sub(tmp, B_w16_fx);
        FOR ( i = 0; i < N; i++)
        {
            IF (Rsubband_fx[i] > 0)
            {
                Rsubband_fx[i] = sub(Rsubband_fx[i], tmp);
                move16();
                BREAK;
            }
        }
    }

    /* Calcuate total used bits and initialize R to be used for Noise Filling */
    tmp = 0;
    move16();
    FOR ( i = 0; i < N; i++)
    {
        tmp = add(tmp, Rsubband_fx[i]);
        R[i] = shr(Rsubband_fx[i], 3);
        move16();
    }
    return shr(tmp, 3);
}
/*-------------------------------------------------------------------*
 * Bit_group()
 *
 * bit allocation in group
 *-------------------------------------------------------------------*/
static
void Bit_group_fx (
    Word16 *y,                 /* i  : norm of sub-band                              Q0*/
    Word16 start_band,         /* i  : start band indices                            Q0*/
    Word16 end_band,           /* i  : end band indices                              Q0*/
    Word16 Bits,               /* i  : number of allocation bits in group            Q0*/
    Word16 thr,                /* i  : smallest bit number for allocation in group   Q0*/
    Word32 *Rsubband_fx,          /* o  : bit allocation of sub-band                    Q21*/
    Word16 *fac_fx                /* i  : weight factor for norm of sub-band            Q13*/
)
{
    Word16 i, j, k, m, y_index[16], index[16], bit_band, band_num, norm_sum;
    Word16 tmp,exp;
    Word16 factor_fx;
    Word32 R_temp_fx[16], R_sum_fx = 0, R_sum_org_fx = 0, Bits_avg_fx = 0;
    Word32 L_tmp;

    /* initialization for bit allocation in one group*/
    tmp = 6554;
    move16();  /*Q15  1/5    */
    if(sub(thr,5) == 0)
    {
        tmp = 6554;
        move16();  /*Q15  1/5    */
    }
    if(sub(thr,6) == 0)
    {
        tmp = 5462;
        move16();/*Q15  1/6 */
    }
    if(sub(thr,7) == 0)
    {
        tmp = 4682;
        move16();/*Q15  1/7 */
    }
    bit_band = mult(tmp, Bits); /*0+15-15=0, Q0 */
    band_num = sub(end_band,start_band);

    FOR( i = 0; i < band_num; i++ )
    {
        y_index[i] = y[add(i,start_band)];
        move16();
        index[i] = i;
        move16();
    }

    /* Rearrange norm vector in decreasing order */
    reordvct_fx(y_index, band_num, index);
    /* norm vector modification */

    factor_fx = div_s(1, band_num);/*Q15 */
    IF ( sub(thr,5) > 0 )
    {
        FOR ( i = 0; i < band_num; i++ )
        {
            L_tmp = L_mult(i,factor_fx);/*Q16 */
            tmp = extract_h(L_shl(L_tmp, 13)); /*Q13 */
            tmp = sub(fac_fx[1],tmp);/*Q13 */
            L_tmp = L_mult(y_index[i],tmp);/*Q14 */
            y_index[i] = extract_h(L_shl(L_tmp, 2));/*Q0 */
        }
    }
    ELSE
    {
        FOR ( i = 0; i < band_num; i++ )
        {
            L_tmp = L_mult(i,factor_fx);/*Q16 */
            tmp = extract_h(L_shl(L_tmp, 13)); /*Q13 */
            tmp = sub(fac_fx[0],tmp);/*Q13 */
            L_tmp = L_mult(y_index[i],tmp);/*Q14 */
            y_index[i] = extract_h(L_shl(L_tmp, 2));/*Q0 */
        }
    }

    /* bit allocation based on modified norm */
    L_tmp = L_mult(band_num,24576);/*Q16 */
    tmp = extract_h(L_shl(L_tmp,7));/*Q7 */
    IF ( sub(shl(bit_band,7),tmp) >= 0 )
    {
        FOR ( j = 0; j < band_num; j++)
        {
            if ( y_index[j] < 0 )
            {
                y_index[j] = 0;
                move16();
            }
            R_temp_fx[j] = 2097152;
            move16();/*Q21 = 1     move16(); */
        }

        i = sub(band_num,1);
        norm_sum = 0;/*Q0 */
        FOR (k = 0; k <= i; k++)
        {
            norm_sum = add(norm_sum,y_index[k]);
        }

        FOR (j = 0; j < band_num; j++)
        {
            IF(norm_sum == 0)
            {
                FOR (k = 0; k <= i; k++)
                {
                    R_temp_fx[k] = L_deposit_h(0);/*Q21 */
                }
            }
            ELSE
            {
                exp = norm_s(norm_sum);
                tmp = shl(norm_sum, exp);/*Q(exp) */
                tmp = div_s(16384,tmp); /*Q(15+14-exp) */
                Bits_avg_fx = L_mult(tmp, Bits);/*Q(30-exp) */

                FOR (k = 0; k <= i; k++)
                {
                    L_tmp = L_shl(L_deposit_l(y_index[k]),24);
                    L_tmp = Mult_32_32(Bits_avg_fx,L_tmp);/*Q(23-exp) */

                    R_temp_fx[k] = L_shl(L_tmp,sub(exp,2));
                    move32();/*Q21 */
                }
            }

            L_tmp = L_shl(L_deposit_l(thr),21);/*Q21 */
            IF ( L_sub(R_temp_fx[i],L_tmp) < 0 )
            {
                R_temp_fx[i] = L_deposit_h(0);
                norm_sum = sub(norm_sum,y_index[i]);
                i--;
            }
            ELSE
            {
                BREAK;
            }
        }
    }
    ELSE
    {
        FOR ( j = 0; j < bit_band; j++ )
        {
            if ( y_index[j] < 0 )
            {
                y_index[j] = 0;
                move16();
            }
            R_temp_fx[j] = 2097152;
            move32();/*Q21 = 1 */
        }

        FOR ( j = bit_band; j < band_num; j++ )
        {
            R_temp_fx[j] = L_deposit_l(0);
        }

        norm_sum = 0;
        FOR (k = 0; k < bit_band; k++)
        {
            norm_sum = add(norm_sum,y_index[k]);
        }

        i = bit_band;
        FOR (j = 0; j < bit_band; j++)
        {
            IF(norm_sum == 0)
            {
                FOR (k = 0; k < i; k++)
                {
                    R_temp_fx[k] = L_deposit_l(0); /*Q21                    */
                }
            }
            ELSE
            {
                exp = norm_s(norm_sum);
                tmp = shl(norm_sum, exp);/*Q(exp) */
                tmp = div_s(16384,tmp); /*Q(15+14-exp) */
                Bits_avg_fx = L_mult(tmp, Bits);/*Q(30-exp) */
                FOR (k = 0; k < i; k++)
                {
                    L_tmp = L_shl(L_deposit_l(y_index[k]),24);
                    L_tmp = Mult_32_32(Bits_avg_fx,L_tmp);/*Q(23-exp) */

                    R_temp_fx[k] = L_shl(L_tmp,sub(exp,2));
                    move32();/*Q21 */
                }
            }
            R_sum_fx = 0;
            L_tmp = L_shl(L_deposit_l(thr),21);/*Q21 */
            FOR (k = 0; k < i; k++)
            {
                IF (L_sub(R_temp_fx[k],L_tmp) < 0)
                {
                    FOR(m = k; m < i; m++)
                    {
                        norm_sum = sub(norm_sum,y_index[m]);
                        R_temp_fx[m] = L_deposit_l(0); /*Q21 */
                    }
                    i = k;
                    BREAK;
                }
                ELSE
                {
                    R_sum_fx = L_add(R_sum_fx,R_temp_fx[k]);
                }
            }
            IF (L_sub(R_sum_fx,R_sum_org_fx) == 0)
            {
                BREAK;
            }

            R_sum_org_fx = R_sum_fx;
        }
    }

    /*  index comeback */
    FOR ( k = 0 ; k < band_num; k++ )
    {
        j = index[k];
        move16();
        Rsubband_fx[add(j,start_band)] = R_temp_fx[k];
        move32();
    }

    return;

}

/*-------------------------------------------------------------------*
 * BitAllocWB()
 *
 * WB bit allocation
 *-------------------------------------------------------------------*/

Word16 BitAllocWB_fx(         /* o  : t                                           Q0*/
    Word16 *y,                /* i  : norm of sub-vectors                         Q0*/
    Word16 B,                 /* i  : number of available bits                    Q0*/
    Word16 N,                 /* i  : number of sub-vectors                       Q0*/
    Word16 *R,                /* o  : bit-allocation indicator                    Q0*/
    Word16 *Rsubband_fx       /* o  : sub-band bit-allocation vector              Q3*/
)
{
    Word16 t_fx;
    Word16 i, j, k, B1, B2, B3, B_saved;
    Word16 Rsum_fx, Rsum_sub_fx[3];
    Word32 Ravg_sub_32_fx[3], R_diff_32_fx[2];
    Word16 factor_fx[2];/*Q13 */
    Word16 BANDS;
    Word16 tmp,exp;
    Word16 Rsum_sub_fx_tmp=0; /* initialize just to avoid compiler warning */
    Word32 L_tmp,L_tmp1;
    Word32 Rsubband_buf[NB_SFM];

    BANDS = N;
    move16();
    if( sub(BANDS,SFM_N) > 0)
    {
        BANDS = SFM_N;
        move16();
    }
    /* Init Rsubband to non-zero values for bands to be allocated bits */
    FOR (k = 0; k < BANDS; k++)
    {
        Rsubband_buf[k] = 2097152;
        move32();/*Q21 */
    }
    /* Calculate the norm sum and average of sub-band */
    Rsum_sub_fx[0] = 0;
    move16();
    FOR ( j = 0; j < SFM_G1; j++ )
    {
        if ( y[j] > 0 )
        {
            Rsum_sub_fx_tmp = add(Rsum_sub_fx[0],y[j]);     /*Q0 */
        }
        if (y[j] > 0)
        {
            Rsum_sub_fx[0] = Rsum_sub_fx_tmp;
            move16();     /*Q0 */
        }
    }
    Ravg_sub_32_fx[0] = L_mult(Rsum_sub_fx[0], 2048);
    move32();/*Q16  0+15+1   //q15 1/16 =2048 */

    Rsum_sub_fx[1] = 0;
    move16();
    FOR ( j = SFM_G1; j < SFM_G1G2; j++ )
    {
        if ( y[j] > 0 )
        {
            Rsum_sub_fx_tmp = add(Rsum_sub_fx[1],y[j]);    /*Q0 */
        }
        if ( y[j] > 0 )
        {
            Rsum_sub_fx[1] = Rsum_sub_fx_tmp;
            move16();/*Q0 */
        }
    }
    Ravg_sub_32_fx[1] = L_mult(Rsum_sub_fx[1], 4096); /*16  0+15+1   //q15 1/8 =4096 */

    Rsum_sub_fx[2] = 0;
    move16();
    FOR ( j = SFM_G1G2; j < BANDS; j++ )
    {
        if ( y[j] > 0 )
        {
            Rsum_sub_fx_tmp = add(Rsum_sub_fx[2],y[j]);    /*Q0 */
        }
        if ( y[j] > 0 )
        {
            Rsum_sub_fx[2] = Rsum_sub_fx_tmp;
            move16();/*Q0 */
        }
    }
    tmp = div_s(1, BANDS-SFM_G1G2); /*Q15 */
    Ravg_sub_32_fx[2] = L_mult(Rsum_sub_fx[2], tmp);
    move32();/*Q16 */

    /* Bit allocation for every group */
    tmp = add(Rsum_sub_fx[0],Rsum_sub_fx[1]);
    Rsum_fx = add(tmp,Rsum_sub_fx[2]);/*Q0     */

    factor_fx[0] = 16384;/*Q13     move16(); */
    factor_fx[1] = 24576;/*Q13     move16(); */
    {
        R_diff_32_fx[0] = L_sub(Ravg_sub_32_fx[0], Ravg_sub_32_fx[1]);
        move32();/*Q16 */
        R_diff_32_fx[1] = L_sub(Ravg_sub_32_fx[1], Ravg_sub_32_fx[2]);
        move32();/*Q16 */

        test();
        IF ( L_sub(R_diff_32_fx[0],393216) < 0 && L_sub(R_diff_32_fx[1],245760) < 0 )
        {
            IF(Rsum_fx == 0)
            {
                B1 = 0;
                move16();
                B2 = 0;
                move16();
                B3 = 0;
                move16();
            }
            ELSE
            {
                exp = norm_s(Rsum_fx);
                tmp = shl(Rsum_fx,exp);/*Q(exp) */
                tmp = div_s(16384,tmp);/*Q(15+14-exp) */
                L_tmp1 = L_mult(B,Rsum_sub_fx[0]);/*Q1 */
                L_tmp = Mult_32_16(L_tmp1,tmp);/*Q(15-exp) */
                B1 = extract_h(L_shl(L_tmp,add(exp,1)));/*Q0 */
                test();
                if(L_sub(L_tmp1,L_mult(B1,Rsum_fx)) > 0 && L_sub(L_tmp1,L_mult(add(B1,1),Rsum_fx)) >= 0)
                {
                    B1 = add(B1,1);
                }
                L_tmp1 = L_mult(B,Rsum_sub_fx[1]);/*Q1 */
                L_tmp = Mult_32_16(L_tmp1,tmp);/*Q(15-exp) */
                B2 = extract_h(L_shl(L_tmp,add(exp,1)));/*Q0 */
                test();
                if(L_sub(L_tmp1,L_mult(B2,Rsum_fx)) > 0 && L_sub(L_tmp1,L_mult(add(B2,1),Rsum_fx)) >= 0)
                {
                    B2 = add(B2,1);
                }
                L_tmp1 = L_mult(B,Rsum_sub_fx[2]);/*Q1 */
                L_tmp = Mult_32_16(L_tmp1,tmp);/*Q(15-exp) */
                B3 = extract_h(L_shl(L_tmp,add(exp,1)));/*Q0 */
                test();
                if(L_sub(L_tmp1,L_mult(B3,Rsum_fx)) > 0 && L_sub(L_tmp1,L_mult(add(B3,1),Rsum_fx)) >= 0)
                {
                    B3 = add(B3,1);
                }
            }
            IF ( L_sub(Ravg_sub_32_fx[2],786432) > 0 )
            {
                B_saved = 0;
                move16();
                IF ( sub(B1,288) > 0 )
                {
                    B_saved = sub(B1,288);
                    B1 = 288;
                    move16();
                }

                IF ( sub(B2,256) > 0 )
                {
                    tmp = sub(B2,256);
                    B_saved = add(B_saved,tmp);
                    B2 = 256;
                    move16();
                }

                IF ( sub(B3,96) > 0 )
                {
                    tmp = sub(B3,96);
                    B_saved = add(B_saved,tmp);
                    B3 = 96;
                    move16();
                }

                IF ( B_saved > 0 )
                {
                    IF ( sub(B1,288) == 0 )
                    {
                        tmp = shr(B_saved,1);
                        B2 = add(B2,tmp);
                        tmp = sub(B,B1);
                        B3 = sub(tmp,B2);
                    }
                    ELSE
                    {
                        tmp = shr(B_saved,1);
                        B1 = add(B1,tmp);
                        IF ( sub(B2,256) == 0 )
                        {
                            tmp = sub(B,B1);
                            B3 = sub(tmp,B2);
                        }
                        ELSE
                        {
                            tmp = sub(B,B1);
                            B2 = sub(tmp,B3);
                        }
                    }
                }
            }

            factor_fx[0] = 16384;
            move16();/*Q13 */
            factor_fx[1] = 12288;
            move16();/*Q13 */
        }
        ELSE
        {
            IF(Rsum_fx == 0)
            {
                B1 = 0;
                move16();
                B2 = 0;
                move16();
                B3 = B;
                move16();
            }
            ELSE
            {
                exp = norm_s(Rsum_fx);
                tmp = shl(Rsum_fx,exp);/*Q(exp) */
                tmp = div_s(16384,tmp);/*Q(15+14-exp) */
                L_tmp1 = L_mult(B,Rsum_sub_fx[0]);/*Q1 */
                L_tmp = Mult_32_16(L_tmp1,tmp);/*Q(15-exp) */
                B1 = extract_h(L_shl(L_tmp,add(exp,1)));/*Q0 */
                test();
                if(L_sub(L_tmp1,L_mult(B1,Rsum_fx)) > 0 && L_sub(L_tmp1,L_mult(add(B1,1),Rsum_fx)) >= 0)
                {
                    B1 = add(B1,1);
                }
                L_tmp1 = Mult_32_16(1975684956,shl(B,5));/*Q(31+5-15=21) */
                L_tmp1 = Mult_32_16(L_tmp1,shl(Rsum_sub_fx[1],7));/*Q(21+7-15=13) */
                L_tmp = Mult_32_16(L_tmp1,tmp);/*Q(27-exp) */
                B2 = extract_h(L_shl(L_tmp,sub(exp,11)));/*Q0 */
                test();
                if(L_sub(L_tmp1,L_shl(L_mult(B2,Rsum_fx),12)) > 0 && L_sub(L_add(L_tmp1,2),L_shl(L_mult(add(B2,1),Rsum_fx),12)) >= 0)
                {
                    B2 = add(B2,1);
                }
                tmp = sub(B,B1);
                B3 = sub(tmp,B2);
            }
        }
    }

    IF ( sub(Rsum_sub_fx[2],3) < 0 )
    {
        B2 = add(B2,B3);
        B3 = 0;
        move16();
    }

    /* Bit allocation in group */
    Bit_group_fx( y, 0, SFM_G1, B1, 5, Rsubband_buf, factor_fx);
    Bit_group_fx( y, SFM_G1, SFM_G1G2, B2, 6, Rsubband_buf, factor_fx);
    Bit_group_fx( y, SFM_G1G2, BANDS, B3, 7, Rsubband_buf, factor_fx);
    FOR (i = 0; i < BANDS; i++)
    {
        Rsubband_fx[i] = extract_l(L_shr(Rsubband_buf[i], 18));
    }

    /* Calcuate total used bits and initialize R to be used for Noise Filling */
    L_tmp = L_deposit_l(0);
    FOR( i = 0; i < N; i++)
    {
        L_tmp = L_add(L_tmp,Rsubband_buf[i]);/*Q21 */
        R[i] = extract_h(L_shr(Rsubband_buf[i],5));/*Q0 */
    }
    t_fx = extract_h(L_shr(L_tmp, 5)); /*Q0 */

    return (Word16)t_fx;

}
