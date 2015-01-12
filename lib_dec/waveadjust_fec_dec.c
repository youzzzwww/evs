/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "stat_com.h"
#include "stl.h"    /* FOR wmc_tool */

void get_maxConv_and_pitch_x(Word16 *s_LP, Word16 s, Word16 e, Word16 N,
                             Word32 *maxConv, Word16 *maxConv_bits, Word16 *pitch);
Word16 get_voicing_x(Word16 *s_LP, Word16 pitch, Word32 covMax,Word16 maxConv_bits, Word16 Framesize);
Word32 con_Log10(Word32 i_s32Val, Word16 i_s16Q);


Word16 vadmin(Word16 a, Word16 b)
{
    return s_min(a, b);
}

void set_state(Word16 *state, Word16 num, Word16 N)
{
    Word16 i, tmp;

    tmp = sub(N, 1);
    FOR (i = 0; i < tmp; i++)
    {
        state[i] = state[i+1];
        move16();
    }
    state[tmp] = num;
    move16();
}

void concealment_update_x(Word16 bfi, Word16 curr_mode, Word16 tonality, Word32 *invkoef, Word16 *invkoef_scale, void *_plcInfo)
{
    T_PLCInfo *plcInfo = (T_PLCInfo*)_plcInfo;
    Word32 *data_reci2 = plcInfo->data_reci2_fx;
    Word16 *tcx_tonality = plcInfo->TCX_Tonality;
    Word16 FrameSize = plcInfo->FrameSize;
    Word16 subframe = plcInfo->subframe_fx;
    Word16 i;
    move16();
    move16();
    IF (sub(curr_mode ,1)==0)
    {
        set_state(plcInfo->Transient, curr_mode, MAX_POST_LEN);

        FOR (i = 0; i < FrameSize; i++)
        {
            data_reci2[i] = invkoef[i];
            move32();

        }
        plcInfo->data_reci2_scale = *invkoef_scale;
        move16();
        IF (!bfi)
        {
            set_state(tcx_tonality, tonality, DEC_STATE_LEN);
        }
    }
    ELSE
    {

        IF (subframe == 0)
        {
            set_state(plcInfo->Transient, curr_mode, MAX_POST_LEN);

            IF (!bfi)
            {
                set_state(tcx_tonality, tonality, DEC_STATE_LEN);
            }
        }
        {

            Word32 *ptr = data_reci2+subframe;
            Word16 FrameSize2 = shr(FrameSize,1);

            FOR (i = 0; i < FrameSize2; i++)
            {
                ptr[i] = invkoef[i];
                move32();
            }

            plcInfo->data_reci2_scale = *invkoef_scale;
            move16();
        }
    }
    return;
}

static Word16 zero_pass_w32_x(Word16 *s, Word16 N)
{
    Word16 i;
    Word32 temp, zp = L_deposit_l(0);

    FOR (i = 1; i < N; i++)
    {
        temp = L_mac0(-1L, s[i],s[i-1]);
        zp = L_sub(zp, L_shr(temp,31));
    }
    return extract_l(zp);
}

Word16 Sqrt_x_fast(Word32 value)
{
    Word16 norm;
    Word16 result, index;

    norm = sub(23, norm_l(value));
    index = extract_l( L_shr_r(value, add(norm, s_and(norm, 1))));
    result = shr(sqrt_table_pitch_search[index], sub(11, shr(add(norm,1),1)));
    return result;
}

Word32 dot_w32_accuracy_x(Word16 *s1, Word16 *s2, Word16 nbits, Word16 N)
{
    Word16 i;
    Word32 eng = L_deposit_l(0), temp;

    FOR (i = 0; i < N; i++)
    {
        temp = L_mult0(s1[i], s2[i]);
        eng = L_add(eng, L_shr(temp,nbits));
    }

    return eng;
}


Word16 int_div_s_x(Word16 a, Word16 b)
{
    Word16 result = 0;
    Word16 norm, left=0, i;
    move16();
    move16();
    test();
    IF (sub(a,b) < 0 || b == 0)
    {
        return 0;
    }
    ELSE
    {
        a = add(a, shr(b,1));
        norm = sub(norm_s(b),norm_s(a));

        FOR (i = norm; i>= 0; i--)
        {
            left = shr(a, i);
            result = shl(result,1);
            IF (sub(left,b) >= 0)
            {
                result = add(result,1);
                left= sub(left, b);
                a   = add(shl(left,i), s_and(a, sub(shl(1,i),1)));
            }
        }
    }

    return result;
}

Word16 GetW32Norm_x(Word32 *s, Word16 N)
{
    Word32 smax = L_deposit_l(0);
    Word16 smax_norm, i;

    FOR (i = 0; i < N; i++)
    {
        smax = L_or(smax, L_abs(s[i]));
    }

    smax_norm = norm_l(smax);

    return smax_norm;
}

Word16 harmo_x(Word32 *X, Word16  Framesize, Word16 pitch)
{
    Word16  h,  k,  result = 0;
    Word32 ener = L_deposit_l(0), ener_harmo = L_deposit_l(0);
    Word16 norm1, d1, d2;
    Word16 ener_w, ener_harmo_w;
    Word16 nbits = sub(15, norm_s(Framesize));
    move16();
    norm1 = GetW32Norm_x(X, Framesize);

    FOR (k = 1; k < 9; k++)
    {
        h = sub(int_div_s_x(extract_l(L_mult(k, Framesize)), pitch),1);

        d1 = extract_h(L_shl(X[h],  norm1));
        d2 = extract_h(L_shl(X[h+1],norm1));

        ener_harmo = L_add(ener_harmo,
                           L_shr(L_mac0(L_mult0(d1, d1), d2, d2),nbits));
    }

    FOR (k = 0; k < Framesize; k++)
    {
        d1 = extract_h(L_shl(X[k],norm1));
        ener = L_add(ener, L_shr(L_mult0(d1, d1),nbits));
    }

    norm1 = norm_l(ener);
    ener_w = extract_h(L_shl(ener,norm1));
    ener_harmo_w = extract_h(L_shl(ener_harmo,norm1));

    IF (L_sub(ener_harmo ,ener)>= 0)
    {
        return 32767;
    }
    test();
    IF ((ener_harmo_w <= 0)||(ener_w <= 0))
    {
        return 0;
    }
    result = div_s(ener_harmo_w, ener_w);
    return result;
}

Word16 get_low_freq_eng_rate_x(Word32 *mdct_data, Word16 curr_mode, Word16 N)
{
    Word16 N1, N2, i;
    Word32 low_eng = L_deposit_l(0), eng = L_deposit_l(0), smax = L_deposit_l(0);
    Word16 nbits, temp, norm = 0;
    move16();
    N1 = 30;
    N2 = N;
    move16();
    move16();
    IF (sub(2 ,curr_mode)==0)
    {
        N1 = shr(30,1);
        N2 = shr(N,1);
    }

    nbits = sub(15, norm_s(N2));

    FOR (i = 0; i < N2; i++)
    {
        smax = L_or(smax, L_abs(mdct_data[i]));
    }

    norm = norm_l(smax);

    FOR (i = 0; i < N1; i++)
    {
        temp = extract_h(L_shl(mdct_data[i], norm));
        low_eng  = L_add(low_eng, L_shr(L_mult0(temp, temp),nbits));
    }

    FOR (i = N1; i < N2; i++)
    {
        temp = extract_h(L_shl(mdct_data[i], norm));
        eng  = L_add(eng, L_shr(L_mult0(temp, temp),nbits));
    }
    eng = L_add(low_eng, eng);

    /* IF (low_eng<(eng+EPSILON)*0.02) return 1;ELSE return 0; */
    /* smax=eng*0.02 */
    smax = L_shr(Mpy_32_16_1(eng, 5243), 3);

    return (L_sub(low_eng,smax) <= 0);
}

void LpFilter2_x(Word16 *x, Word16 *y, Word16 N)
{
    Word16  i;
    Word16 smax, norm;
    Word16 a1 = 5898;  /* W16(0.18f); */
    Word16 a2 = 20971; /* W16(0.64f); */
    move16();
    move16();
    smax=0;
    move16();
    FOR (i = 0; i < N; i++)
    {
        smax = s_or(smax, abs_s(x[i]));
    }
    norm = norm_s(smax);

    y[0] = mult(shl(x[0],norm), a1);
    move16();
    y[1] = add(mult(y[0], a2),mult(shl(x[1],norm), a1));
    move16();

    FOR (i = 2; i < N; i++)
    {
        /* 5898*2+20971=32767 -->no overflow */
        y[i] = add(mult(y[i-2],a1), add(mult(y[i-1],a2), mult(shl(x[i],norm),a1)));
        move16();
    }
}

void sig_tilt_x(Word16 *s, Word16 FrameSize, Word32 *enr1, Word32 *enr2)
{
    Word16 subFrameSize, shIFt;
    Word16 *p1, *p2;
    Word16 nbits;

    subFrameSize = shr(FrameSize, 2);
    p1 = s+subFrameSize;
    p2 = s+sub(subFrameSize, 2);

    shIFt = sub(FrameSize, subFrameSize);
    nbits = sub(15, norm_s(shIFt));
    *enr1 = dot_w32_accuracy_x(p1, p2, nbits, shIFt);
    move32();
    *enr2 = dot_w32_accuracy_x(p1, p1, nbits, shIFt);
    move32();
}

void get_maxConv_and_pitch_x(Word16 *s_LP, Word16 s, Word16 e, Word16 N,
                             Word32 *maxConv, Word16 *maxConv_bits, Word16 *pitch)
{
    Word16 t, cov_size, size = N;
    Word32 tmp_sigma = L_deposit_l(0), cov_max_sigma = L_deposit_l(0);
    Word16 nbits,tmp_pitch=0, Lowbits;
    Word32 r1_high, r2_high;
    Word32 r1_low, r2_low;
    move16();
    move16();
    nbits = sub(15, norm_s(sub(N, s)));

    FOR (t = s; t < e; t++)
    {
        cov_size =  sub(N,t);
        tmp_sigma = dot_w32_accuracy_x(s_LP, s_LP+t, nbits, cov_size);

        Lowbits = extract_l(L_shr(L_and(tmp_sigma,  0x0000ffff), 1));
        r1_low = L_mult0(Lowbits, size);

        r1_high = L_shr( r1_low, 14 );
        r1_high = L_mac( r1_high, size, extract_h(tmp_sigma) ) ;

        Lowbits = extract_l(L_shr(L_and(cov_max_sigma,  0x0000ffff), 1));
        r2_low = L_mult0(Lowbits, cov_size);
        r2_high = L_shr( r2_low, 14 );
        r2_high = L_mac( r2_high, cov_size, extract_h(cov_max_sigma) ) ;

        IF(L_sub(r1_high ,r2_high)>0)
        {
            cov_max_sigma = L_add(tmp_sigma, 0);
            size = cov_size;
            move16();
            tmp_pitch = t;
            move16();
        }
        ELSE
        {
            test();
            IF((L_sub(r1_high ,r2_high)==0)
            && (L_sub(L_and(r1_low,  0x7fff),L_and(r2_low,  0x7fff))>0))
            {
                cov_max_sigma = L_add(tmp_sigma, 0);
                size = cov_size;
                move16();
                tmp_pitch = t;
                move16();
            }
        }
    }

    *pitch = tmp_pitch;
    move16();
    *maxConv = cov_max_sigma;
    move32();
    *maxConv_bits = nbits;
    move16();
}

Word16 get_voicing_x(Word16 *s_LP, Word16 pitch, Word32 covMax,Word16 maxConv_bits,  Word16 Framesize)
{
    Word32 eng1 = L_deposit_l(0), eng2 = L_deposit_l(0);
    Word16 voicing, norm;
    Word16 tmpLen, nbits;
    Word16 eng1_w, eng2_w;

    IF (covMax <= 0)
    {
        return 0;
    }
    ELSE
    {
        tmpLen = sub(Framesize, pitch);
        nbits = maxConv_bits;
        move16();

        eng1 = dot_w32_accuracy_x(s_LP, s_LP, nbits, tmpLen);
        eng2 = dot_w32_accuracy_x(s_LP+pitch, s_LP+pitch, nbits, tmpLen);

        norm  = sub(norm_l(L_or(eng1, L_or(eng2, covMax))),1);
        eng1   = L_shl(eng1,   norm);
        eng2   = L_shl(eng2,   norm);
        covMax = L_shl(covMax, norm);

        eng1_w = Sqrt_x_fast(eng1);
        eng2_w = Sqrt_x_fast(eng2);

        eng1 = L_mult0(eng1_w, eng2_w);
        norm = norm_l(eng1);
        eng1_w = extract_h(L_shl(eng1, norm));
        eng2_w = extract_h(L_shl(covMax, norm));

        IF (L_sub(covMax , eng1)>=0)
        {
            return 32767;
        }
        test();
        IF ((eng2_w <= 0)||(eng1_w <= 0))
        {
            return 0;
        }
        voicing = div_s(eng2_w, eng1_w);

        return voicing;
    }
}

void pitch_modify_x(Word16 *s_LP, Word16 *voicing, Word16 *pitch, Word16 FrameSize)
{
    Word32 eng1, eng2, eng3;
    Word16 shIFt = shr(*pitch ,1);
    Word16 tmpLen, nbits, norm, voicing2;
    Word16 eng1_w, eng2_w;

    tmpLen = sub(FrameSize, shIFt);
    nbits = sub(15, norm_s(tmpLen));

    eng1 = dot_w32_accuracy_x(s_LP+shIFt, s_LP+shIFt, nbits, tmpLen);
    eng2 = dot_w32_accuracy_x(s_LP, s_LP, nbits, tmpLen);
    eng3 = dot_w32_accuracy_x(s_LP+shIFt, s_LP, nbits, tmpLen);

    IF (eng3 <= 0)
    {
        return ;
    }

    norm  = sub(norm_l(L_or(eng1, L_or(eng2, eng3))),1);
    eng1   = L_shl(eng1,   norm);
    eng2   = L_shl(eng2,   norm);
    eng3   = L_shl(eng3,   norm);

    eng1_w = Sqrt_x_fast(eng1);
    eng2_w = Sqrt_x_fast(eng2);

    eng1 = L_mult0(eng1_w, eng2_w);

    norm = norm_l(eng1);
    eng1_w = extract_h(L_shl(eng1, norm));
    eng2_w = extract_h(L_shl(eng3, norm));

    IF (L_sub(eng3,eng1) >= 0)
    {
        voicing2 = 32767;
        move16();
    }
    ELSE {                                                                            test();
            IF ((eng2_w <= 0)||(eng1_w <= 0))
    {
        voicing2 = 0;
        move16();
    }
    ELSE {
        voicing2 = div_s(eng2_w, eng1_w);
    }
         }

    IF (sub(voicing2, *voicing) > 0)
    {
        *pitch = shIFt;
        move16();
        *voicing = voicing2;
        move16();
    }
}

Word16 Is_Periodic_x(Word32 *mdct_data, Word16 cov_max, Word16  zp, Word32 ener,
                     Word32 ener_mean, Word16  pitch, Word16  Framesize)
{
    Word16 flag =0;
    Word16 harm;
    move16();
    test();
    test();
    test();
    IF (L_sub(ener, L_shl(50,8)) < 0 || (L_sub(ener, L_sub(ener_mean, L_shl(8,8))) < 0
                                         && sub(cov_max, 29491) < 0 ))
    {
        flag = 0;
        move16();
    }
    ELSE IF (sub(cov_max, 26214) > 0)
    {
        flag = 1;
        move16();
    }
    ELSE IF (sub(zp, 100) > 0)
    {
        flag = 0;
        move16();
    }
    ELSE IF (L_sub(ener, L_sub(ener_mean,L_shl(6,8))) < 0)
    {
        flag = 0;
        move16();
    }
    ELSE IF (L_sub(ener, L_add(ener_mean, L_shl(1,8))) > 0 && sub(cov_max, 19661) > 0)
    {
        flag = 1;
        move16();
    }
    ELSE
    {
        harm = harmo_x(mdct_data, Framesize, pitch);
        flag = 1;
        move16();
        if (sub(harm, 22938) < 0)
        {
            flag = 0;
            move16();
        }
    }

    return flag;
}

Word16 get_conv_relation_x(Word16 *s_LP, Word16 shIFt, Word16 N)
{
    Word32 eng1,eng2, eng3;
    Word16 eng1_w, eng2_w;
    Word16 tmp, norm, nbits;

    nbits = sub(15, norm_s(N));
    eng3 = dot_w32_accuracy_x(s_LP, s_LP+shIFt, nbits, N);

    IF (eng3 <= 0)
    {
        return 0;
    }

    eng1 = dot_w32_accuracy_x(s_LP+shIFt, s_LP+shIFt, nbits, N);
    eng2 = dot_w32_accuracy_x(s_LP, s_LP, nbits, N);

    norm = sub(norm_l(L_or(eng1, L_or(eng2, eng3))),1);
    eng1 = L_shl(eng1,   norm);
    eng2 = L_shl(eng2,   norm);
    eng3 = L_shl(eng3,   norm);

    eng1_w = Sqrt_x_fast(eng1);
    eng2_w = Sqrt_x_fast(eng2);

    eng1 = L_mult0(eng1_w, eng2_w);

    norm = norm_l(eng1);
    eng1_w = extract_h(L_shl(eng1, norm));
    eng2_w = extract_h(L_shl(eng3, norm));

    IF (L_sub(eng3, eng1) >= 0)
    {
        return 32767;
    }
    test();
    IF ((eng2_w <= 0)||(eng1_w <= 0))
    {
        return 0;
    }

    tmp = div_s(eng2_w, eng1_w);

    return tmp;
}

Word16  pitch_search_fx(Word16 *s,
                        Word16 *outx_new,
                        Word16 Framesize,
                        Word16 *voicing,
                        Word16 zp,
                        Word32 ener,
                        Word32 ener_mean,
                        Word32 *mdct_data,
                        Word16 curr_mode)
{
    Word16 pitch = 0;
    Word32 cov_max = L_deposit_l(0), tilt_enr1, tilt_enr2;
    Word16 s_LP[L_FRAME_MAX];
    Word16 start_pos, end_pos;
    Word16 low_freq_rate_result;
    Word16 flag = 0, zp_current;
    Word32 *mdctPtr;
    Word16 curr_frmsize, cov_max_bits=0;
    Word16 i;
    move16();
    move16();
    move16();
    *voicing = 0;
    move16();
    curr_frmsize = Framesize;
    move16();
    if (sub(2, curr_mode) == 0)
    {
        curr_frmsize = shr(Framesize, 1);
    }

    zp_current = zero_pass_w32_x(outx_new, curr_frmsize);
    IF (sub(curr_frmsize, 256) <= 0)
    {
        IF (sub(zp_current, 70) > 0)
        {
            return 0;
        }
    }
    ELSE IF(sub(curr_frmsize, 320) == 0)
    {
        IF (sub(zp_current, 105) > 0)
        {
            return 0;
        }
    }
    ELSE /* curr_frmsize == 512 and above */
    {
        IF (sub(zp_current, 210) > 0)
        {
            return 0;
        }
    }

    mdctPtr = mdct_data;
    if (sub(2, curr_mode) == 0)
    {
        mdctPtr = mdct_data + shr(Framesize,1);
    }

    low_freq_rate_result = get_low_freq_eng_rate_x(mdctPtr, curr_mode, Framesize);

    IF(low_freq_rate_result)
    {
        return 0;
    }

    LpFilter2_x(s, s_LP, Framesize);
    sig_tilt_x(s_LP, Framesize, &tilt_enr1, &tilt_enr2);
    IF (sub(Framesize, 320) <= 0)
    {
        test();
        IF ((0==tilt_enr2) ||
            (L_sub(tilt_enr1, L_shr(tilt_enr2, 1)) < 0))
        {
            return 0;
        }
    }
    ELSE
    {
        test();
        IF ((0==tilt_enr2) ||
        (L_sub(tilt_enr1, Mpy_32_16_1(tilt_enr2, 22938)) < 0))
        {
            return 0;
        }
    }

    IF (sub(Framesize, 320) <= 0)
    {
        start_pos = extract_l(L_shr(L_mac0(0x80, 34, Framesize), 8));
        end_pos = extract_l(L_shr(L_mac0(0x2, 3, Framesize), 2));
        get_maxConv_and_pitch_x(s_LP, start_pos, end_pos, Framesize, &cov_max, &cov_max_bits, &pitch);
        *voicing = get_voicing_x(s_LP, pitch, cov_max, cov_max_bits, Framesize);
        move16();
        pitch_modify_x(s_LP, voicing, &pitch, Framesize);
    }
    ELSE
    {
        Word16 s_tmp[L_FRAME_MAX];
        Word16 Framesize_tmp;
        Word16 pitch_tmp[3];
        Word16 cov_size;

        Framesize_tmp = shr(Framesize, 1);
        FOR (i = 0; i < Framesize_tmp; i++)
        {
            s_tmp[i] = s_LP[2*i];
            move16();
        }

        start_pos = extract_l( L_shr(L_mac0(0x80, 34, Framesize_tmp), 8));
        end_pos = extract_l( L_shr(L_mac0(0x2, 3, Framesize_tmp), 2));

        cov_max = L_deposit_l(0);
        pitch = 0;
        move16();
        get_maxConv_and_pitch_x(s_tmp, start_pos, end_pos, Framesize_tmp, &cov_max, &cov_max_bits, &pitch);

        pitch_tmp[0] = 0;
        move16();
        if (sub(shl(pitch, 1), 1) > 0)
        {
            pitch_tmp[0] = sub(shl(pitch, 1), 1);
            move16();
        }
        pitch_tmp[1] = shl(pitch, 1);
        move16();
        pitch_tmp[2] = add(shl(pitch, 1), 1);
        move16();
        start_pos = 0;
        move16();
        pitch = 0;
        move16();
        FOR (i = 0; i < 3; i++)
        {
            cov_size =  sub(Framesize, pitch_tmp[i]);
            end_pos = get_conv_relation_x(s_LP, pitch_tmp[i], cov_size);
            IF (sub(end_pos, start_pos) > 0)
            {
                start_pos = end_pos;
                move16();
                pitch = pitch_tmp[i];
                move16();
            }
        }
        *voicing = start_pos;
        move16();
    }

    IF (pitch > 0)
    {
        flag = Is_Periodic_x(mdct_data, *voicing, zp, ener, ener_mean, pitch, Framesize);
    }
    if (flag == 0 )
    {
        pitch = 0;
        move16();
    }
    return pitch;
}

void concealment_init_x(Word16 N, void *_plcInfo)
{
    T_PLCInfo *plcInfo = (T_PLCInfo*)_plcInfo;
    Word16 i;

    plcInfo->FrameSize = N;
    move16();
    plcInfo->Pitch_fx = 0;
    move16();
    plcInfo->T_bfi_fx = 0;
    move16();
    plcInfo->outx_new_n1_fx = 0;
    move16();
    plcInfo->nsapp_gain_fx = 0;
    move16();
    plcInfo->nsapp_gain_n_fx = 0;
    move16();
    plcInfo->ener_mean_fx = L_deposit_l(15213);                                                  /*Q8 59.4260f*256*/
    plcInfo->ener_fx = L_deposit_l(0);
    plcInfo->zp_fx = N;
    move16();
    plcInfo->recovery_gain = 0;
    move16();
    plcInfo->step_concealgain_fx = 0;
    move16();
    plcInfo->concealment_method = TCX_NONTONAL;
    move16();
    plcInfo->subframe_fx = 0;
    move16();
    plcInfo->nbLostCmpt = L_deposit_l(0);
    plcInfo->seed = 21845;
    move16();

    FOR (i = 0; i < DEC_STATE_LEN; i++)
    {
        plcInfo->TCX_Tonality[i] = 0;
        move16();
    }

    FOR (i = 0; i < MAX_POST_LEN; i++)
    {
        plcInfo->Transient[i] = 0;
        move16();
    }

    FOR (i = 0; i < L_FRAME_MAX; i++)
    {
        plcInfo->data_reci2_fx[i] = L_deposit_l(0);
    }
    return;
}

static Word16 own_random_fix(                         /* o  : output random value */
    Word16 *seed            /* i/o: random seed         */
)
{
    *seed = extract_l(L_mac0(13849L, *seed , 31821));
    return(*seed);
}

void concealment_decode_fix(Word16 curr_mode, Word32 *invkoef, Word16 *invkoef_scale,void *_plcInfo)
{
    T_PLCInfo *plcInfo = (T_PLCInfo*)_plcInfo;
    Word16 i;
    Word16 N = plcInfo->FrameSize;
    Word16 *seed = &(plcInfo->seed);
    Word16 sign;
    move16();
    IF (plcInfo->concealment_method == TCX_NONTONAL)    /* #define TCX_NONTONAL 0 */
    {
        IF (sub(curr_mode, 1) == 0)
        {
            /* copy the data of the last frame */
            mvr2r_Word32(plcInfo->data_reci2_fx, invkoef, N);
            *invkoef_scale = plcInfo->data_reci2_scale;
            move16();
            /* sign randomization */
            FOR (i = 0; i < N; i++)
            {
                sign = add(shl(shr(own_random_fix(seed),15),1),1);
                if(sub(sign,-1)==0)
                {
                    invkoef[i] = L_negate(invkoef[i]);
                    move32();
                }
            }
        }
    }
    return;
}


Word16 Spl_GetScalingSquare_x(Word16 *in_vector, Word16 in_vector_length, Word16 times)
{
    Word16 nbits = sub(15, norm_s(times))/*Spl_GetSizeInBits_x(times)*/;
    Word16 i;
    Word16 smax = -1;
    Word16 sabs;
    Word16 *sptr = in_vector;
    Word16 t;
    move16();
    FOR (i = in_vector_length; i > 0; i--)
    {

        sabs = abs_s(*sptr);
        sptr++;
        smax = s_max(sabs, smax);

    }

    t = norm_l(L_mult0(smax, smax));

    IF (smax == 0)
    {
        return 0; /* Since norm(0) returns 0 */
    }
    ELSE
    {
        nbits = sub(nbits, t);
        nbits = s_max(0,nbits);

        return nbits;
    }
}


Word32 Spl_Energy_x(Word16* vector, Word16 vector_length, Word16* scale_factor)
{
    Word32 en = L_deposit_l(0);
    Word32 i;
    Word16 scaling = Spl_GetScalingSquare_x(vector, vector_length, vector_length);

    FOR (i = 0; i < vector_length; i++)
    {
        en = L_add(en,L_shr(L_mult0(vector[i], vector[i]), scaling));
    }

    move32();
    *scale_factor = scaling;

    return en;
}

void Log10OfEnergy_x(Word16 *s, Word32 *enerlogval, Word16 len)
{
    Word32 energy = L_deposit_l(0), tmp2 = L_deposit_l(0);
    Word16 shfts = 0;
    Word32 Log10_energy = L_deposit_l(0), Log10_len = L_deposit_l(0);
    move16();
    energy = Spl_Energy_x(s, len, &shfts);/* Q:-shfts */
    IF (energy > 0)
    {
        Log10_energy = con_Log10(energy, negate(shfts)); /* Q25 */
        Log10_len = con_Log10(L_deposit_l(len), 0); /* Q25 */
        tmp2 = L_sub(Log10_energy,Log10_len); /* Q25 */
        tmp2 = Mpy_32_16_1(tmp2,20480); /* Q11->10   Q=25+11-15=21 */
        *enerlogval = L_shr(tmp2,13); /* Q8 */                                    move32();
    }
    ELSE
    {
        *enerlogval = -25600;
        move32();
    }

}

static Word32 fnLog2(Word32 L_Input)
{

    Word16 swC0 = -0x2b2a, swC1 = 0x7fc5, swC2 = -0x54d0;
    Word16 siShIFtCnt, swInSqrd, swIn;
    Word32 LwIn;
    move16();
    move16();
    move16();
    /*_________________________________________________________________________
     |                                                                         |
     |                              Executable Code                            |
     |_________________________________________________________________________|
    */

    /* normalize input and store shIFts required */
    /* ----------------------------------------- */

    siShIFtCnt = norm_l(L_Input);
    LwIn = L_shl(L_Input, siShIFtCnt);
    siShIFtCnt = add(siShIFtCnt, 1);
    siShIFtCnt = negate(siShIFtCnt);

    /* calculate x*x*c0 */
    /* ---------------- */

    swIn = extract_h(LwIn);
    swInSqrd = mult_r(swIn, swIn);
    LwIn = L_mult(swInSqrd, swC0);

    /* add x*c1 */
    /* --------- */

    LwIn = L_mac(LwIn, swIn, swC1);

    /* add c2 */
    /* ------ */

    LwIn = L_add(LwIn, L_deposit_h(swC2));

    /* apply *(4/32) */
    /* ------------- */

    LwIn = L_shr(LwIn, 3);
    LwIn = L_and(LwIn, 0x03ffffff);
    siShIFtCnt = shl(siShIFtCnt, 10);
    LwIn = L_add(LwIn, L_deposit_h(siShIFtCnt));

    /* return log2 */
    /* ----------- */

    return (LwIn);
}

static Word32 fnLog10(Word32 L_Input)
{

    Word16 Scale = 9864;            /* 0.30103 = log10(2) */
    Word32 LwIn;
    move16();
    /*_________________________________________________________________________
     |                                                                         |
     |                              Executable Code                            |
     |_________________________________________________________________________|
    */

    /* 0.30103*log2(x) */
    /* ------------------- */

    LwIn = fnLog2(L_Input);
    LwIn = Mpy_32_16_1(LwIn, Scale);

    return (LwIn);
}

Word32 con_Log10(Word32 i_s32Val, Word16 i_s16Q)
{
    Word32 s32Out;
    Word32 s32Correct;                                           /* corrected (31-q)*log10(2) */
    const Word16 s16Log10_2 = 19728;                             /* log10(2)~Q16 */   move16();

    IF(0 == i_s32Val)
    {
        return EVS_LW_MIN;
    }

    s32Out = fnLog10(i_s32Val);                                  /* (2^26)*log10(a) */

    s32Correct = L_mult(sub(31,i_s16Q), s16Log10_2);             /* q = 17 */
    s32Correct = L_shl(s32Correct, 8);                           /* q = 25 */
    s32Out = L_shr(s32Out, 1);                                   /* q = 25 */

    s32Out = L_add(s32Out, s32Correct);

    return s32Out;
}

void concealment_update2_x(Word16 *outx_new, void *_plcInfo, Word16 FrameSize)
{
    T_PLCInfo *plcInfo = (T_PLCInfo*)_plcInfo;

    plcInfo->zp_fx = zero_pass_w32_x(outx_new, FrameSize);
    move16();

    Log10OfEnergy_x(outx_new, &plcInfo->ener_fx, FrameSize); /* Q8 */
    test();
    IF (sub(plcInfo->zp_fx, 100) < 0 && L_sub(plcInfo->ener_fx, L_shl(50,8)) > 0)
    {
        plcInfo->ener_mean_fx  = L_add(Mpy_32_16_1(plcInfo->ener_mean_fx ,FIX_16(0.98f)),
                                       Mpy_32_16_1(plcInfo->ener_fx , FIX_16(0.02f)));
        move32();
    }
    return;
}

static Word16 array_max_indx_fx(Word16 *s, Word16 N)
{
    Word16 i, indx = 0;
    move16();
    FOR (i = 0; i < N; i++)
    {
        if (sub(s[i], s[indx]) > 0)
        {
            indx = i;
            move16();
        }
    }
    return indx;
}

Word16 ffr_getSfWord16(                 /* o: measured headroom in range [0..15], 0 IF all x[i] == 0 */
    Word16 *x,      /* i: array containing 16-bit data */
    Word16 len_x)   /* i: length of the array to scan  */
{
    Word16 i, i_min, i_max;
    Word16 x_min, x_max;


    x_max = 0;
    move16();
    x_min = 0;
    move16();
    FOR (i = 0; i < len_x; i++)
    {
        if (x[i] >= 0)
            x_max = s_max(x_max,x[i]);
        if (x[i] < 0)
            x_min = s_min(x_min,x[i]);
    }

    i_max = 0x10;
    move16();
    i_min = 0x10;
    move16();

    if (x_max != 0)
        i_max = norm_s(x_max);

    if (x_min != 0)
        i_min = norm_s(x_min);

    i = s_and(s_min(i_max, i_min),0xF);


    return i;
}

static Word16 OverlapAdd_fx(Word16 *pitch125_data, Word16 *sbuf,
                            Word16  n, Word16  pitch, Word16  Framesize)
{
    Word16 n1,n2,s,s16MaxCoefNorm,s16MaxCoefNorm2,tmp16;
    Word16 i;
    Word16 pitch125 =extract_l(L_shr(L_add(L_add(L_shl(pitch,16), L_mult(pitch, 8192)), 32768) ,16));
    Word16 Loverlap = sub(pitch125,pitch);
    Word16 tmp =  sub(Framesize,n);

    n1 = tmp;
    move16();
    n2 = tmp;
    move16();
    if(sub(Loverlap,tmp)<0)
    {
        n1 = Loverlap;
        move16();
    }
    if(sub(pitch125,tmp)<0)
    {
        n2 = pitch125;
        move16();
    }
    s16MaxCoefNorm = sub(ffr_getSfWord16(sbuf+n, n1),1);
    s16MaxCoefNorm2 = ffr_getSfWord16(pitch125_data, n1);
    tmp16 =BASOP_Util_Divide1616_Scale(1, Loverlap,&s);
    FOR (i = 0; i < n1; i++)
    {
        Word16 tmp;
        Word16 dat;
        dat= shl(sbuf[n+i],s16MaxCoefNorm);
        tmp = extract_l(L_shl(L_mult0(i, tmp16), s)); /* q15 */
        sbuf[n+i] = round_fx(L_add(L_shr(L_mult(dat, sub(32767,tmp)),s16MaxCoefNorm),
                                   L_shr(L_mult(shl(pitch125_data[i],s16MaxCoefNorm2),tmp),s16MaxCoefNorm2)));
    }

    FOR (i = n1; i < n2; i++)
    {
        sbuf[n+i] = pitch125_data[i];
        move16();
    }

    pitch = add(n, pitch);

    return pitch;
}

Word16 waveform_adj_fix(Word16 *overlapbuf,
                        Word16 *outdata2,
                        Word16 *outx_new,
                        Word16 *data_noise,
                        Word16 *outx_new_n1,
                        Word16 *nsapp_gain,
                        Word16 *nsapp_gain_n,
                        Word16 Framesize,
                        Word8  pre_bfi,
                        Word16 voicing,
                        Word16 curr_mode,
                        Word16 subframe,
                        Word16 pitch)
{
    Word16 i, zp1, zp2,Framesizediv2,s16MaxCoefNorm;
    Word16 sbuf[L_FRAME_MAX];
    Word16 tmp;

    Framesizediv2=shr(Framesize,1);
    test();
    IF ((sub(curr_mode, 1)==0) || subframe == 0)
    {
        zp1 = zero_pass_w32_x(outdata2, Framesizediv2);
        zp2 = zero_pass_w32_x(outdata2+Framesizediv2, Framesizediv2);

        /* judge if the pitch is usable */
        tmp = 1;
        move16();
        if (sub(zp1, 1) > 0)
        {
            tmp = zp1;
            move16();
        }
        IF (sub(shl(tmp,2), zp2) < 0)
        {
            move16();
            return pitch = 0;
        }

        /* adjust the pitch value */
        test();
        test();
        test();
        IF (pre_bfi && (sub(pitch , Framesizediv2)<=0)
            && (sub(Framesize ,256)>0) && (sub(curr_mode , 1)==0))
        {
            Word16 i1 = 0, i2 = 0;
            Word16  pos1, pos2, pos3;
            move16();
            move16();
            i1 = add(1 , array_max_indx_fx(outx_new, pitch));
            i2 = add(1 , array_max_indx_fx(outx_new+pitch, pitch));

            pos1 = add(i2,sub(pitch,i1));
            pos3 = add(pos1, mult(pos1, FIX_16(0.25f)));
            pos2 = add(pitch,mult(pitch, FIX_16(0.25f)));

            test();
            test();
            IF ((sub(pos1,pos2)<0) && (sub(pos3,pitch)>0) && (sub(pos1,Framesizediv2)<0))
            {
                pitch = add(i2,sub(pitch,i1));
            }
        }

        {
            Word16 pitch125 = 0, Loverlap = 0, n = 0, i;
            Word16 pitch125_data[L_FRAME_MAX];
            move16();
            move16();
            move16();
            Loverlap = sub(pitch125 ,pitch);
            pitch125 = extract_l((L_shr(L_add(L_add(L_shl(pitch,16), L_mult(pitch, 8192)), 32768) ,16)));
            Loverlap = sub(pitch125,pitch);
            FOR (i = 0; i < pitch; i++)
            {
                pitch125_data[i] = outdata2[Framesize-pitch+i];
                move16();
            }
            FOR (i = 0; i < Loverlap; i++)
            {
                pitch125_data[pitch+i] = outx_new[i];
                move16();
            }
            FOR (i = 0; i < Framesize; i++)
            {
                sbuf[i] = outx_new[i];
                move16();
            }

            {
                Word16  i,pitch125a1;
                Word16 tmp[2*L_FRAME_MAX], *p_tmp = tmp+1;

                FOR (i = 0; i < pitch125; i++)
                {
                    p_tmp[i] = pitch125_data[i];
                    move16();
                }

                p_tmp[-1] = outdata2[Framesize-pitch-1];
                move16();
                p_tmp[pitch125] = outx_new[Loverlap];
                move16();
                pitch125a1 = add(pitch125,1);
                s16MaxCoefNorm = sub(ffr_getSfWord16(p_tmp-1, pitch125a1),1);
                FOR (i = 0; i < pitch125a1; i++)
                {
                    p_tmp[i-1] = shl(p_tmp[i-1],s16MaxCoefNorm);
                    move16();
                }
                FOR (i = 0; i < pitch125; i++)
                {
                    pitch125_data[i] = round_fx(L_shr(L_add((L_mult(p_tmp[i], 20972)),L_mac(L_mult(p_tmp[i-1], 5898),p_tmp[i+1],5898)),s16MaxCoefNorm));
                }
            }

            WHILE (sub(n, Framesize) < 0)   /* periodical extension */
            {
                n = OverlapAdd_fx(pitch125_data,sbuf,n,pitch,Framesize);
            }

            FOR (i = 0; i < Framesize; i++)
            {
                overlapbuf[i] = sbuf[i];
                move16();
            }
            {
                /* add some noise */

                Word16 gain_n,temp_OUT;
                Word16 gain = 0;
                move16();
                gain_n = sub(32767 ,shr(voicing,1)); /* q15 */
                mvr2r_Word16(outdata2, data_noise, Framesize);
                IF (sub(curr_mode, 1) == 0)
                {
                    mvr2r_Word16(outx_new, data_noise+Framesize, Framesize);
                }
                ELSE
                {
                    mvr2r_Word16(outx_new, data_noise+Framesize, Framesizediv2);
                    FOR (i = 0; i < Framesizediv2; i++)
                    {
                        data_noise[Framesize+Framesizediv2+i] = outx_new[Framesizediv2-i-1];
                        move16();
                    }
                }

                IF (sub(curr_mode , 1)==0)
                {
                    FOR (i = 1; i < Framesize; i++)
                    {
                        temp_OUT = sub(data_noise[i], mult(data_noise[i-1],FIX_16(0.68f)));
                        sbuf[i] = add(sbuf[i], mult(temp_OUT, gain));
                        move16();
                        gain = mac_r(L_mult(FIX_16(0.99f),gain),FIX_16(0.01f),gain_n);
                    }
                    *outx_new_n1 = data_noise[Framesize-1];
                    move16();
                }
                ELSE
                {
                    FOR (i = 1; i < Framesizediv2; i++)
                    {
                        temp_OUT = sub(data_noise[i], mult(data_noise[i-1],FIX_16(0.68f)));
                        sbuf[i] = add(sbuf[i], mult(temp_OUT, gain));
                        move16();
                        gain = mac_r(L_mult(FIX_16(0.99f),gain),FIX_16(0.01f),gain_n);
                    }
                    *outx_new_n1 = data_noise[Framesizediv2-1];
                    move16();
                }
                *nsapp_gain = gain;
                move16();
                *nsapp_gain_n = gain_n;
                move16();
            }
        }
    }
    ELSE   /* processing in case of the second subframe */
    {
        Word16 gain_n,temp_OUT;
        Word16 gain = 0;
        Word16* noise_seg = data_noise + Framesizediv2;
        move16();
        mvr2r_Word16(overlapbuf+Framesizediv2, sbuf, Framesizediv2);
        gain = *nsapp_gain;
        move16();
        gain_n = *nsapp_gain_n;
        move16();
        temp_OUT = sub(noise_seg[0], mult((*outx_new_n1),FIX_16(0.68f)));
        sbuf[0] = add(sbuf[0], mult((temp_OUT), gain));
        move16();
        gain = add(mult(gain, FIX_16(0.99f)), mult(gain_n, FIX_16(0.01f)));
        FOR(i = 1; i < Framesizediv2; i++)
        {
            temp_OUT = sub(noise_seg[i], mult((noise_seg[i-1]),FIX_16(0.68f)));
            sbuf[i] = add(sbuf[i], mult((temp_OUT), gain));
            move16();
            gain = mac_r(L_mult(FIX_16(0.99f),gain),FIX_16(0.01f),gain_n);
        }
        *outx_new_n1 = noise_seg[Framesizediv2-1]; /*q0*/                             move16();
        *nsapp_gain = gain; /*q15*/                                                   move16();
        *nsapp_gain_n = gain_n; /*q15*/                                               move16();
    }

    IF (sub(curr_mode , 1)==0)
    {
        FOR (i = 0; i < Framesize; i++)
        {
            outx_new[i] = sbuf[i];
            move16();
        }
    }
    ELSE
    {
        FOR (i = 0; i < Framesizediv2; i++)
        {
            outx_new[i] = sbuf[i];
            move16();
        }
    }
    return pitch;
}

void waveform_adj2_fix(  Word16 *overlapbuf,
                         Word16 *outx_new,
                         Word16 *data_noise,
                         Word16 *outx_new_n1,
                         Word16 *nsapp_gain,
                         Word16 *nsapp_gain_n,
                         Word16 *recovery_gain,
                         Word16 step_concealgain,
                         Word16 pitch,
                         Word16 Framesize,
                         Word16 curr_mode,
                         Word16 subframe,
                         Word16 bfi_cnt,
                         Word16  bfi
                      )
{
    Word16 i, n,tablescale,ratio,coutlength,dat,gain,gain_n,Framesizesubn,Framesizesubp,tmp16,s,ptable,temp_OUT,Framesizediv2,s16MaxCoefNorm,s16MaxCoefNorm2;
    Word16 sbuf[L_FRAME_MAX];

    n=0;
    move16();
    Framesizediv2=shr(Framesize,1);
    Framesizesubn = sub(Framesize,n);
    Framesizesubp = sub(Framesize,pitch);
    IF (pitch > 0)
    {
        test();
        IF (sub(curr_mode,1)==0 || subframe == 0)
        {
            WHILE (Framesizesubn>0)
            {
                /* periodical extension */
                Word16 tmp = vadmin(pitch, Framesizesubn);
                FOR (i = 0; i < tmp; i++)
                {
                    sbuf[n+i] = overlapbuf[Framesizesubp+i];
                    move16();
                }
                n = add(n, pitch);
                Framesizesubn = sub(Framesize,n);
            }

            FOR (i = 0; i < Framesize; i++)
            {
                overlapbuf[i] = sbuf[i];
                move16();
            }
        }
        ELSE   /* processing in case of the second subframe */
        {
            mvr2r_Word16(overlapbuf+Framesizediv2, sbuf, Framesizediv2);
        }

        gain = *nsapp_gain;
        move16();
        gain_n = *nsapp_gain_n;
        move16();
        IF (sub(bfi_cnt, 2) == 0)
        {
            IF (sub(curr_mode, 1) == 0)
            {
                Word16 *noise_seg = data_noise + Framesize;
                temp_OUT = sub(noise_seg[0], mult((*outx_new_n1),FIX_16(0.68f)));
                sbuf[0] = add(sbuf[0], mult((temp_OUT), gain));
                move16();
                gain = add(mult(gain, FIX_16(0.99f)), mult(gain_n, FIX_16(0.01f)));

                FOR (i = 1; i < Framesize; i++)
                {
                    temp_OUT = sub(noise_seg[i], mult(noise_seg[i-1],FIX_16(0.68f)));
                    sbuf[i] = add(sbuf[i], mult(temp_OUT, gain));
                    move16();
                    gain = mac_r(L_mult(FIX_16(0.99f),gain),FIX_16(0.01f),gain_n);
                }
                *outx_new_n1 = noise_seg[Framesize-1];
                move16();
            }
            ELSE
            {
                Word16 *noise_seg = data_noise + Framesize+ subframe;
                temp_OUT = sub(noise_seg[0], mult((*outx_new_n1),FIX_16(0.68f)));
                sbuf[0] = add(sbuf[0], mult((temp_OUT), gain));
                move16();
                gain = add(mult(gain, FIX_16(0.99f)), mult(gain_n, FIX_16(0.01f)));
                FOR (i = 1; i < Framesizediv2; i++)
                {
                    temp_OUT = sub(noise_seg[i], mult(noise_seg[i-1],FIX_16(0.68f)));
                    sbuf[i] = add(sbuf[i], mult(temp_OUT, gain));
                    move16();
                    gain = mac_r(L_mult(FIX_16(0.99f),gain),FIX_16(0.01f),gain_n);
                }
                *outx_new_n1 = noise_seg[Framesizediv2-1];
                move16();
            }
        }
        ELSE
        {
            IF (sub(curr_mode, 1) == 0)
            {
                Word16 *noise_seg;
                temp_OUT = sub(bfi_cnt,3);
                noise_seg = data_noise + sub(shl(Framesize,1),1) - L_mult0(temp_OUT,Framesize);
                temp_OUT = sub(noise_seg[0], mult((*outx_new_n1),FIX_16(0.68f)));
                sbuf[0] = add(sbuf[0], mult((temp_OUT), gain));
                move16();
                gain = add(mult(gain, FIX_16(0.99f)), mult(gain_n, FIX_16(0.01f)));

                FOR (i = 1; i < Framesize; i++)
                {
                    temp_OUT = sub(noise_seg[-i], mult(noise_seg[-i+1],FIX_16(0.68f)));
                    sbuf[i] = add(sbuf[i], mult(temp_OUT, gain));
                    move16();
                    gain = mac_r(L_mult(FIX_16(0.99f),gain),FIX_16(0.01f),gain_n);
                }
                *outx_new_n1 = noise_seg[-Framesize+1];
                move16();
            }
            ELSE{
                Word16 *noise_seg;
                temp_OUT = sub(bfi_cnt,3);
                noise_seg = data_noise + sub(shl(Framesize,1),1) - L_mult0(temp_OUT,Framesize)- subframe;
                temp_OUT = sub(noise_seg[0], mult((*outx_new_n1),FIX_16(0.68f)));
                sbuf[0] = add(sbuf[0], mult((temp_OUT), gain));
                move16();
                gain = add(mult(gain, FIX_16(0.99f)), mult(gain_n, FIX_16(0.01f)));
                FOR (i = 1; i < Framesizediv2; i++)
                {
                    temp_OUT = sub(noise_seg[-i], mult(noise_seg[-i+1],FIX_16(0.68f)));
                    sbuf[i] = add(sbuf[i], mult(temp_OUT, gain));
                    move16();
                    gain = mac_r(L_mult(FIX_16(0.99f),gain),FIX_16(0.01f),gain_n);
                }
                *outx_new_n1 = noise_seg[-Framesizediv2+1];
                move16();
            }
        }
        *nsapp_gain = gain;
        move16();
        *nsapp_gain_n = gain_n;
        move16();
        test();
        IF (sub(bfi_cnt ,4)==0 || bfi == 0)
        {
            SWITCH ( Framesize)
            {
            case 320:
                {
                    tablescale =9;
                    move16();
                    ptable = 26214; /* (Word16)(32767*256/320.0+0.5); q8+15 */        move16();
                    BREAK;
                }
            case 512:
                {
                    tablescale =10;
                    move16();
                    ptable = 32767;        /* q9+15 */                                move16();
                    BREAK;
                }
            case 640:
                {
                    tablescale =10;
                    move16();
                    ptable = 26214; /* (Word16)(32767*512/640.0+0.5); q9+15 */        move16();
                    BREAK;
                }
            default:  /* 960  */
                {
                    tablescale =10;
                    move16();
                    ptable = 17456; /* (Word16)(32767*512/960.0); q9+15    */         move16();
                    BREAK;
                }
            }
            IF (bfi == 0)    /* overlap-and-add */
            {
                Word16 gain_zero_start = 10000;
                move16();

                IF (sub(curr_mode,1)==0)
                {
                    IF (step_concealgain > 0)
                    {
                        gain_zero_start = BASOP_Util_Divide1616_Scale(*recovery_gain, step_concealgain,&s);
                        gain_zero_start= shl(gain_zero_start, sub(s,15)); /* q0 */
                        gain_zero_start= add(gain_zero_start,1);
                    }
                    s16MaxCoefNorm = sub(ffr_getSfWord16(sbuf, Framesize),1);
                    s16MaxCoefNorm2 = ffr_getSfWord16(outx_new, Framesize);

                    tmp16 = vadmin(gain_zero_start, Framesize);
                    FOR (i = 0; i < tmp16; i++)
                    {
                        ratio = extract_l(L_shr(L_mult(i, ptable), tablescale));
                        dat= shl(sbuf[i],s16MaxCoefNorm);
                        temp_OUT= mult(*recovery_gain, sub(32767,ratio));
                        outx_new[i]= round_fx(L_add(L_shr(L_mult(temp_OUT,dat ),s16MaxCoefNorm), L_shr(L_mult(shl(outx_new[i],s16MaxCoefNorm2),ratio),s16MaxCoefNorm2)));
                        move16();
                        *recovery_gain =sub(*recovery_gain,step_concealgain);
                        move16();

                    }
                    FOR (i = gain_zero_start; i < Framesize; i++)
                    {
                        ratio = extract_l(L_shr(L_mult(i, ptable), tablescale));
                        outx_new[i] = round_fx(L_shr(L_mult(shl(outx_new[i],s16MaxCoefNorm2),ratio),s16MaxCoefNorm2));

                    }

                    if (*recovery_gain < 0)
                    {
                        *recovery_gain = 0;
                        move16();
                    }
                }
                ELSE
                {
                    IF (step_concealgain > 0)
                    {
                        gain_zero_start = BASOP_Util_Divide1616_Scale(*recovery_gain, step_concealgain,&s);
                        gain_zero_start= shl(gain_zero_start, sub(s,15)); /* q0 */
                        gain_zero_start= add(gain_zero_start,1);
                        gain_zero_start= add(gain_zero_start,subframe);
                    }

                    s16MaxCoefNorm = sub(ffr_getSfWord16(sbuf, Framesizediv2),1);
                    s16MaxCoefNorm2 = ffr_getSfWord16(outx_new, Framesizediv2);
                    tmp16 = add(subframe,Framesizediv2);
                    tmp16 = vadmin(gain_zero_start, tmp16);

                    FOR (i = subframe; i < tmp16; i++)
                    {
                        ratio = extract_l(L_shr(L_mult(i, ptable), tablescale));
                        dat= shl(sbuf[i-subframe],s16MaxCoefNorm);
                        temp_OUT= mult(*recovery_gain, sub(32767,ratio));
                        outx_new[i-subframe]= round_fx(L_add(L_shr(L_mult(temp_OUT,dat),s16MaxCoefNorm), L_shr(L_mult(shl(outx_new[i-subframe],s16MaxCoefNorm2),ratio),s16MaxCoefNorm2)));
                        *recovery_gain =sub(*recovery_gain,step_concealgain);
                        move16();
                    }

                    tmp16 = add(subframe,Framesizediv2);
                    FOR (i = gain_zero_start; i < tmp16; i++)
                    {
                        ratio = extract_l(L_shr(L_mult(i, ptable), tablescale));
                        outx_new[i-subframe] = round_fx(L_shr(L_mult(shl(outx_new[i-subframe],s16MaxCoefNorm2),ratio),s16MaxCoefNorm2));
                    }

                    if (*recovery_gain < 0)
                    {
                        *recovery_gain = 0;
                        move16();
                    }
                }
            }
            ELSE
            {
                /* overlap-and-add */
                Word16 tmp;
                Word16 dat;
                IF (sub(curr_mode, 1)==0)
                {
                    s16MaxCoefNorm = sub(ffr_getSfWord16(sbuf, Framesize),1);
                    s16MaxCoefNorm2 = ffr_getSfWord16(outx_new, Framesize);
                    FOR (i = 0; i < Framesize; i++)
                    {
                        dat= shl(sbuf[i],s16MaxCoefNorm);
                        tmp = extract_l(L_shr(L_mult(i, ptable), tablescale));
                        outx_new[i] = round_fx(L_add(L_shr(L_mult(dat, sub(32767,tmp)),s16MaxCoefNorm), L_shr(L_mult(shl(outx_new[i],s16MaxCoefNorm2),tmp),s16MaxCoefNorm2)));
                    }
                }
                ELSE {
                    s16MaxCoefNorm = sub(ffr_getSfWord16(sbuf, Framesizediv2),1);
                    s16MaxCoefNorm2 = ffr_getSfWord16(outx_new, Framesizediv2);
                    coutlength = add(subframe,Framesizediv2);
                    FOR (i = subframe; i < coutlength; i++)
                    {
                        dat= shl(sbuf[i-subframe],s16MaxCoefNorm);
                        tmp = extract_l(L_shr(L_mult(i, ptable), tablescale));
                        outx_new[i-subframe] = round_fx(L_add(L_shr(L_mult(dat, sub(32767,tmp)),s16MaxCoefNorm), L_shr(L_mult(shl(outx_new[i-subframe],s16MaxCoefNorm2),tmp),s16MaxCoefNorm2)));
                    }
                }
            }
        }
        ELSE
        {
            IF (sub(curr_mode, 1)==0)
            {
                FOR (i = 0; i < Framesize; i++)
                {
                    outx_new[i] = sbuf[i];
                    move16();
                }
            }
            ELSE {
                FOR (i = 0; i < Framesizediv2; i++)
                {
                    outx_new[i] = sbuf[i];
                    move16();
                }
            }
        }
    }
    return;
}

void concealment_signal_tuning_fx(Word16 bfi, Word16 curr_mode, Word16 *outx_new_fx, void *_plcInfo, Word16 nbLostCmpt, Word16 pre_bfi, Word16 *OverlapBuf_fx, Word16 past_core_mode, Word16 *outdata2_fx, Decoder_State_fx *st)
{
    T_PLCInfo *plcInfo = (T_PLCInfo*)_plcInfo;
    Word16    FrameSize  = plcInfo->FrameSize;
    Word16    Pitch       = plcInfo->Pitch_fx;
    Word16    subframe    = plcInfo->subframe_fx;
    Word16    voicing_fx  = 0;
    move16();
    move16();
    move16();
    move16();
    IF (bfi)
    {

        test();
        IF (st->enablePlcWaveadjust && plcInfo->concealment_method == TCX_NONTONAL)   /* #define TCX_NONTONAL 0 */
        {

            IF (sub(nbLostCmpt, 1) == 0)
            {
                test();
                IF (sub(curr_mode, 1) == 0 || subframe == 0)
                {
                    plcInfo->Pitch_fx = pitch_search_fx(outdata2_fx,
                                                        outx_new_fx,
                                                        FrameSize,
                                                        &voicing_fx,
                                                        plcInfo->zp_fx,
                                                        (plcInfo->ener_fx),
                                                        (plcInfo->ener_mean_fx),
                                                        plcInfo->data_reci2_fx,
                                                        curr_mode);
                    move16();
                }

                IF (plcInfo->Pitch_fx)    /* waveform adjustment for the first lost frame */
                {
                    plcInfo->Pitch_fx = waveform_adj_fix(OverlapBuf_fx,
                                                         outdata2_fx,
                                                         outx_new_fx,
                                                         plcInfo->data_noise,
                                                         &plcInfo->outx_new_n1_fx,
                                                         &plcInfo->nsapp_gain_fx,
                                                         &plcInfo->nsapp_gain_n_fx,
                                                         FrameSize,
                                                         plcInfo->T_bfi_fx,
                                                         voicing_fx,
                                                         curr_mode,
                                                         plcInfo->subframe_fx,
                                                         plcInfo->Pitch_fx);
                    move16();
                }
            }
            ELSE IF (sub(nbLostCmpt, 5) < 0)    /* waveform adjustment for the 2nd~4th lost frame */
            {
                waveform_adj2_fix(OverlapBuf_fx,
                                  outx_new_fx,
                                  plcInfo->data_noise,
                                  &plcInfo->outx_new_n1_fx,
                                  &plcInfo->nsapp_gain_fx,
                                  &plcInfo->nsapp_gain_n_fx,
                                  &plcInfo->recovery_gain,
                                  plcInfo->step_concealgain_fx,
                                  Pitch,
                                  FrameSize,
                                  curr_mode,
                                  plcInfo->subframe_fx,
                                  nbLostCmpt,
                                  bfi);
            }
        }
        plcInfo->T_bfi_fx = 1;
        move16();
    }
    ELSE
    {
        test();
        test();
        test();
        IF (pre_bfi && past_core_mode != 0 && !bfi && L_sub(st->last_total_brate_fx, 48000) >= 0)
        {
            IF (plcInfo->concealment_method == TCX_NONTONAL)    /* #define TCX_NONTONAL 0 */
            {
                IF (L_sub(plcInfo->nbLostCmpt, 4) < 0)   /* smoothing of the concealed signal with the good signal */
                {
                    waveform_adj2_fix(OverlapBuf_fx,
                    outx_new_fx,
                    plcInfo->data_noise,
                    &plcInfo->outx_new_n1_fx,
                    &plcInfo->nsapp_gain_fx,
                    &plcInfo->nsapp_gain_n_fx,
                    &plcInfo->recovery_gain,
                    plcInfo->step_concealgain_fx,
                    Pitch,
                    FrameSize,
                    curr_mode,
                    plcInfo->subframe_fx,
                    add(extract_l(plcInfo->nbLostCmpt), 1),
                    bfi);
                }
            }
        }
        ELSE {
            plcInfo->T_bfi_fx = 0;
            move16();
        }
    }
    return;
}
