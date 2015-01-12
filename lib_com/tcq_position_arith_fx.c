/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "options.h"     /* Compilation switches                   */
#include "stl.h"

#include "rom_com_fx.h"
#include "prot_fx.h"

Word32 ar_div(Word32 num, Word32 denum)
{
    Word16 exp1, exp2, exp, i;
    Word32 varout;

    test();
    IF (L_sub(num, denum) < 0 || denum == 0)
    {
        return 0;
    }
    ELSE IF (L_sub(num, denum) == 0)
    {
        return 1;
    }
    ELSE
    {
        exp1 = norm_l(num);
        exp2 = norm_l(denum);
        exp = sub(exp2, exp1);
        denum = L_shl(denum, exp);
        exp = add(exp, 1);
        varout = L_deposit_l(0);
        FOR (i = 0; i < exp; i++)
        {
            num = L_sub(num, denum);
            varout = L_shl(varout, 1);
            IF (num >= 0)
            {
                num = L_shl(num, 1);
                varout = L_add(varout, 1);
            }
            ELSE
            {
                num = L_add(num, denum);
                num = L_shl(num, 1);
            }
        }
    }
    return varout;
}

static void bitstream_save_bit(PBITSTREAM_FX pBS, Word32 bit)
{
    UWord8 cur;

    cur = pBS->buf[pBS->numByte];
    move16();


    cur = (UWord8)(cur | (bit << pBS->curPos--));
    move16();
    move16();
    move16();
    pBS->buf[pBS->numByte] = cur;
    move16();
    pBS->numbits = L_add(pBS->numbits, 1);

    IF (pBS->curPos < 0)
    {
        pBS->curPos = 7;
        move16();
        pBS->numByte = L_add(pBS->numByte, 1);
    }

    return;
}

static UWord32 bitstream_load_bit(PBITSTREAM_FX pBS)
{
    UWord32 bit;
    Word16 *curPos;

    curPos = &pBS->curPos;
    move16();
    bit = (( pBS->buf[pBS->numByte] >> (*curPos)--) & 0x00000001);
    move16();
    move16();
    move16();

    IF (*curPos < 0)
    {
        pBS->numByte = L_add(pBS->numByte, 1);
        *curPos = 7;
        move16();
    }

    return bit;
}

static void bitstream_rollback(PBITSTREAM_FX pBS, Word32 numBits)
{

    FOR ( ; numBits > 0; numBits--)
    {
        pBS->curPos++;
        move16();
        pBS->numbits = L_sub(pBS->numbits, 1);
        IF (sub(pBS->curPos, 8) == 0)
        {
            pBS->curPos = 0;
            move16();
            pBS->numByte = L_sub(pBS->numByte, 1);
        }
    }

    return;
}
static void transmission_bits( PARCODEC_FX arInst, Word32 bit )
{
    bitstream_save_bit( arInst->bsInst, bit );
    arInst->num_bits = L_add(arInst->num_bits, 1);
    bit = !bit;
    move32();

    FOR ( ; arInst->bits_to_follow > 0;  arInst->bits_to_follow --)
    {
        bitstream_save_bit( arInst->bsInst, bit );
        arInst->num_bits = L_add(arInst->num_bits, 1);
    }

    return;
}

static Word32 ar_make_model_fx( const Word16 *freq, Word16 *model, Word16 len )
{
    Word16 dist;
    Word32 sum = 0;
    Word32 cum = 0;
    Word16 i, tmp;

    FOR ( i = 0 ; i < len ; i ++ )
    {
        sum = L_add(sum, freq[i]);
    }

    IF ( sum == 0 )
    {
        return 0;
    }

    FOR ( i = len ; i >= 0; i -- )
    {
        /*model[i] = (short)( ( cum * MAX_AR_FREQ ) / sum ); */
        model[i] = extract_l(ar_div(cum * MAX_AR_FREQ, sum));

        if (i) cum = L_add(cum, freq[i - 1]);
    }

    tmp = sub(len, 1);
    FOR ( i = 0 ; i < tmp ; i ++ )
    {
        dist = sub(model[i], model[i + 1]);

        IF ( dist <= 0  )
        {
            model[i + 1] = add(model[i + 1], sub(dist, 1));
            move16();
        }
    }

    FOR ( i = len ; i > 0 ; i -- )
    {
        dist = sub(model[i - 1], model[i]);

        IF ( dist <= 0  )
        {
            model[i - 1] = sub(model[i - 1], sub(dist, 1));
            move16();
        }
    }

    return (model[0] > model[1]);
}

void ar_encoder_start_fx( PARCODEC_FX arInst, PBITSTREAM_FX bsInst )
{
    arInst->bsInst = bsInst;
    move32();

    arInst->low  = L_deposit_l(0);
    arInst->high = AR_TOP;
    move32();
    arInst->bits_to_follow  = 0;
    move16();

    arInst->num_bits = L_deposit_l(0);
}

static void ar_encode_fx( PARCODEC_FX arInst, Word16 const *model, Word32 symbol )
{
    Word32 range, high, low;

    high = L_add(arInst->high, 0);
    low = L_add(arInst->low, 0);

    symbol = L_add(symbol, 1);
    range  = L_add(L_sub(high, low), 1);

    high = L_sub(L_add(low, ar_div(range * model[symbol - 1], model[0])), 1);
    low = L_add(low, ar_div(range * model[symbol], model[0]));

    FOR( ; ; )
    {
        IF ( L_sub(high, AR_HALF) < 0 )
        {
            transmission_bits( arInst, 0 );
        }
        ELSE
        {
            IF (L_sub(low, AR_HALF) >= 0 )
            {
                transmission_bits( arInst, 1 );

                low  = L_sub(low, AR_HALF);
                high = L_sub(high, AR_HALF);
            }
            ELSE
            {
                test();
                IF (L_sub(low, AR_FIRST) >= 0 && L_sub(high, AR_THIRD) < 0 )
                {
                    arInst->bits_to_follow ++;
                    move16();

                    low  = L_sub(low, AR_FIRST);
                    high = L_sub(high, AR_FIRST);
                }
                ELSE
                {
                    BREAK;
                }
            }
        }

        low  = L_shl(low, 1);
        high = L_add(L_shl(high, 1 ), 1);
    }

    arInst->high = high;
    move32();
    arInst->low = low;
    move32();

    return;
}

static void ar_encode_uniform_fx( PARCODEC_FX arInst, UWord32 data, Word32 bits )
{
    Word32 i;

    FOR ( i = 0 ; i < bits ; i ++ )
    {
        ar_encode_fx( arInst, uniform_model_fx, data & 0x1 );
        data = L_lshr(data, 1);
    }

    return;
}

void ar_encoder_done_fx( PARCODEC_FX arInst )
{
    arInst->bits_to_follow ++;
    move16();
    transmission_bits( arInst, arInst->low >= AR_FIRST );

    return;
}

void ar_decoder_start_fx( PARCODEC_FX arInst, PBITSTREAM_FX bsInst )
{
    Word16 i;

    arInst->bsInst  = bsInst;
    move32();

    arInst->low    = L_deposit_l(0);
    arInst->high  = AR_TOP;
    move32();
    arInst->value  = L_deposit_l(0);

    FOR ( i = 0; i < AR_BITS ; i ++ )
    {
        arInst->value = L_add(L_shl( arInst->value, 1), bitstream_load_bit( arInst->bsInst ));
    }

    return;
}

static Word16 ar_decode_fx( PARCODEC_FX arInst, Word16 const *model )
{
    Word32 range, high, low, value, i;
    Word16 cum;
    Word16 symbol;

    high = L_add(arInst->high, 0);
    low = L_add(arInst->low, 0);
    value = L_add(arInst->value, 0);

    range = L_add(L_sub( high, low ), 1);
    /*cum   = (short)( ( ( (unsigned int)( arInst->value - arInst->low ) + 1 ) * model[0] - 1 ) / range ); */
    cum   = extract_l(ar_div(L_sub(L_add(L_sub( value, low ), 1 ) * model[0], 1), range));

    symbol = 1;
    move16();
    WHILE ( sub(model[symbol], cum) > 0 )
    {
        symbol = add(symbol, 1);
    }

    high  = L_sub(L_add(low, ar_div(range * model[symbol - 1], model[0])), 1);
    low    = L_add(low, ar_div(range * model[symbol], model[0]));

    FOR (i = 0; i < 0x7FFF; i++)
    {
        Word32 L_msb_diff, L_msb_low, L_msb_high;

        L_msb_high = L_shr(high,14);
        L_msb_low  = L_shr(low,14);
        L_msb_diff = L_sub(L_msb_high, L_msb_low);
        IF (L_sub(L_msb_diff,2) >= 0)
        {
            BREAK;
        }
        assert (tab_ari_qnew[L_msb_high][L_msb_low] != 0x0CCC);
        assert (tab_ari_qnew[L_msb_high][L_msb_low] != 0x0BBB);
        low   = L_msu(low,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        low   = L_shl(low,1);
        high  = L_msu(high,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        high   = L_add(L_shl(high, 1), 1);
        value = L_msu(value,1,tab_ari_qnew[L_msb_high][L_msb_low]);
        value  = L_add(L_shl(value, 1), bitstream_load_bit( arInst->bsInst ));
    }

    arInst->low = low;
    move32();
    arInst->high = high;
    move32();
    arInst->value = value;
    move32();

    return (symbol - 1);
}

void ar_decoder_done_fx( PARCODEC_FX arInst )
{
    bitstream_rollback( arInst->bsInst, AR_BITS - 2 );

    return;
}

static Word16 quantize_fx( Word16 val, Word16 D)
{
    Word16 qval4_fx;
    Word16 retval_fx;

    val = s_min(val, 32255); /* limit input by (2^15 - 1)-2^9 */
    qval4_fx = shr(abs_s(add(val, 512)), 12);
    retval_fx = add(shl(qval4_fx, 2), DDP_fx[D]);
    /* 2nd zero check */
    IF (D == 0)
    {
        if (sub(abs_s(sub(shl(abs_s(retval_fx), 10), abs_s(val))), abs_s(val)) > 0)
        {
            retval_fx = 0;
            move16();
        }
    }
    /*return retval; */
    return retval_fx;
}

static Word32 GetBitsFromPulses_fx(Word16 m, Word16 n)
{
    Word16 i, tmp, integer_fx, temp_fx1, temp_fx2, exp1, exp2;
    Word32 temp32;
    Word32 frac_fx32;
    Word32 logCoeff_fx;
    Word16 exp = 0;
    Word32 mantissa_fx = 0;
    move16();
    move32();

    IF (m == 0)
    {
        return 0;
    }

    tmp = s_min(m, n);
    FOR (i = 0; i < tmp; i++)
    {
        logCoeff_fx = L_add(L_shl(i + 1, 16), L_sub(table_logcum_fx[n+1], L_add(table_logcum_fx[i + 2], table_logcum_fx[n - i])));
        logCoeff_fx = L_add(logCoeff_fx, L_sub(table_logcum_fx[m], L_add(table_logcum_fx[i + 1], table_logcum_fx[m - i])));/*Q16 */
        integer_fx = extract_h(logCoeff_fx);/*Q0 */
        frac_fx32 = L_sub(logCoeff_fx, L_shl(integer_fx, 16));/*Q16 */

        /*ln2, 0.987, ln2 * ln2, 0.977 */
        /*temp1 = (int) (frac / 0.0625); */
        /*temp2 = frac - (float)temp1 * 0.0625f; */

        /* frac = pow(2.0, temp1 * 0.0625) * (1 + 0.693 * temp2 + 0.480 * temp2 * temp2 * 0.5);*/
        /*frac = pow_getbitsfrompulses[temp1] * (1 + 0.693f * temp2 + 0.480f * temp2 * temp2 * 0.5f); */

        temp_fx1 = extract_h(L_shl(frac_fx32, 4));
        temp_fx2 = extract_l(L_and(frac_fx32, 0xfff));/*Q16   */

        frac_fx32 =L_mac(Mult_32_16(L_mult0(temp_fx2, temp_fx2), 7864), temp_fx2, 22708);/*Q32 */
        frac_fx32 = L_add(0x40000000, L_shr(frac_fx32, 2));/*30 */

        exp1 = norm_l(pow_getbitsfrompulses_fx[temp_fx1]);
        exp2 = norm_l(frac_fx32);
        frac_fx32 = Mult_32_32(L_shl(pow_getbitsfrompulses_fx[temp_fx1], exp1), L_shl(frac_fx32, exp2));/*21 + exp1 + 30 + exp2 - 31 */
        frac_fx32 = L_shr(frac_fx32, exp1 + exp2) + 1;/*20 */

        IF (sub(exp, integer_fx) < 0)
        {
            mantissa_fx = L_shr(mantissa_fx, sub(integer_fx, exp));
            mantissa_fx = L_add(mantissa_fx, frac_fx32);

            exp = integer_fx;
            move16();
        }
        ELSE
        {
            mantissa_fx = L_add(mantissa_fx, L_shr(frac_fx32, sub(exp, integer_fx)));
        }
        IF (L_sub(mantissa_fx, 0x200000) >= 0)
        {
            exp++;
            move16();

            mantissa_fx = L_shr(mantissa_fx, 1);
        }
    }

    mantissa_fx = L_shl(mantissa_fx, 2);/*22 */
    temp_fx1 = extract_h(mantissa_fx);
    /*temp_fx2 = extract_h(L_shl(L_sub(mantissa_fx, L_deposit_h(temp_fx1)), 15)); // 15 */
    temp32 = L_shl(L_sub(mantissa_fx, L_deposit_h(temp_fx1)), 15);/*31 */
    exp1 = sub(norm_l(temp32), 1);
    temp32 = ar_div(L_shl(temp32, exp1), temp_fx1); /*31 + exp1 */
    temp32 = L_shr(temp32, exp1 + 1);/*30 */

    frac_fx32 = L_sub(0x40000000, L_shr(temp32, 1));/*30 */
    frac_fx32 = Mult_32_32(frac_fx32, temp32);/*29 */
    frac_fx32 = L_shr(frac_fx32, 13);/*16 */
    exp1 = norm_l(temp_fx1);
    temp_fx1 = Log2_norm_lc(L_shl(temp_fx1, exp1));/*15 */
    return L_add(L_deposit_h(exp), L_add(L_shl(temp_fx1, 1), frac_fx32));
    /*return exp + (float)temp_fx1 / pow(2.0, 15) + (float)frac_fx32 / pow(2.0, 16); */
}

static
void TCQnew_fx( Word32 *v_fx, Word32 scale_fx, Word16 Qscale,
                Word16 length, Word16 *vout_fx, Word16 pulses, Word16 *pulsesout, Word16* nzposout, Word32 *savedstates, Word32 * lasttrellislevel, Word32 terminate)
{
    Word16 i, st, dminpos, position;
    Word16 pulsesnum, nzpos = 0;

    Word32 metric_fx[STATES][TCQ_MAX_BAND_SIZE];
    Word16 path_fx[STATES][TCQ_MAX_BAND_SIZE];
    Word16 quant_fx[STATES][TCQ_MAX_BAND_SIZE];
    Word16 pused_fx[STATES][TCQ_MAX_BAND_SIZE];
    Word16 quantum1_fx, quantum2_fx, newdist1_fx, newdist2_fx/*, signq_fx*/;
    Word32 /*scale_fx, */tmp32, dmin_fx, curdist1_fx, curdist2_fx;
    Word16 value_fx, exp;
    Word16 exp1, exp2;

    set32_fx(*metric_fx, 0, STATES * TCQ_MAX_BAND_SIZE);
    set16_fx( *path_fx, 0, STATES*TCQ_MAX_BAND_SIZE );
    set16_fx( *quant_fx, 0, STATES*TCQ_MAX_BAND_SIZE );
    set16_fx( *pused_fx, 0, STATES*TCQ_MAX_BAND_SIZE );

    /* Initialize metric */
    FOR ( st = 1; st < STATES; st++)
    {
        metric_fx[st][0] = MAX_32>>1;
        move32();
    }
    FOR ( st = 2; st < STATES; st++)
    {
        metric_fx[st][1] = MAX_32>>1;
        move32();
    }
    FOR ( st = 4; st < STATES; st++)
    {
        metric_fx[st][2] = MAX_32>>1;
        move32();
    }
    /* Viterbi for input sequence */
    FOR ( i = 0; i < length; i++) /* cycle over symbols */
    {
        FOR ( st = 0; st < STATES; st++) /* cycle over conditions */
        {
            curdist1_fx = L_add(metric_fx[ step_tcq_fx[st][0]][i], 0);
            curdist2_fx = L_add(metric_fx[ step_tcq_fx[st][1]][i], 0);

            exp = norm_l(scale_fx);
            tmp32 = L_shl(scale_fx, exp);
            tmp32 = Mult_32_16(v_fx[i], extract_h(tmp32));/*12 + 20 + exp - 16 - 15 + Qscale */
            exp1 = 26-(exp-19+Qscale);
            exp2 = norm_l(tmp32);
            IF( sub(exp2, exp1) >= 0 )
            {
                value_fx = extract_h(L_shl(tmp32, sub(26, add(sub(exp, 19), Qscale))));/*exp -19 + Qscale*/ /*10*/
            }
            ELSE
            {
                value_fx = extract_h(L_shl(tmp32, exp2));/*exp -19 + Qscale*/ /*10*/
            }
            /* step 1 */
            quantum1_fx = quantize_fx(value_fx, denc_fx[st][0]);
            newdist1_fx = sub(shl(quantum1_fx, 10), abs_s(value_fx));/*10*/
            newdist1_fx = mult(newdist1_fx, newdist1_fx); /* 5 */

            test();
            if (sub(add(quantum1_fx , pused_fx[step_tcq_fx[st][0]][i]), pulses) > 0 && terminate)
            {
                newdist1_fx = MAX_16;
                move16();
            }
            /* step 2 */
            quantum2_fx = quantize_fx(value_fx, denc_fx[st][1]);
            newdist2_fx = sub(shl(quantum2_fx, 10), abs_s(value_fx));/*10*/
            newdist2_fx = mult(newdist2_fx, newdist2_fx);/*5*/

            test();
            if (sub(add(quantum2_fx , pused_fx[step_tcq_fx[st][1]][i]), pulses) > 0 && terminate)
            {
                newdist2_fx = MAX_16;
                move16();
            }

            /* decision */
            IF (L_sub(L_add(curdist1_fx, newdist1_fx), L_add(curdist2_fx, newdist2_fx)) < 0)
            {
                path_fx[st][i+1] = step_tcq_fx[st][0];
                move16();
                metric_fx[st][i+1] = L_add(curdist1_fx, newdist1_fx);
                move32();
                quant_fx[st][i+1] = quantize_fx(value_fx, denc_fx[st][0]);
                move16();
                pused_fx[st][i+1] = add(pused_fx[ step_tcq_fx[st][0]][i], abs_s( quant_fx[st][i+1] ));
                move16();
            }
            ELSE
            {
                path_fx[st][i+1] = step_tcq_fx[st][1];
                move16();
                metric_fx[st][i+1] = L_add(curdist2_fx, newdist2_fx);
                move32();
                quant_fx[st][i+1] = quantize_fx(value_fx, denc_fx[st][1]);
                move16();
                pused_fx[st][i+1] = add(pused_fx[ step_tcq_fx[st][1]][i], abs_s( quant_fx[st][i+1] ));
                move16();
            }
        }
    }

    /* Find path with minimal metric */
    dminpos = 0;
    move16();
    dmin_fx    = L_add(metric_fx[dminpos][ length], 0);
    FOR ( i = 1; i < STATES; i++)
    {
        test();
        test();
        test();
        IF ( (L_sub(dmin_fx, metric_fx[ i][ length]) > 0 && sub(pused_fx[i][ length], pulses) == 0) ||
             (sub(pused_fx[dminpos][ length], pulses) != 0 && sub(pused_fx[i][ length], pulses) == 0) )
        {
            dmin_fx = L_add(metric_fx[ i][ length], 0);
            dminpos = i;
            move16();
        }
    }
    /* Trace back to get output */
    nzpos = 0;
    move16();
    pulsesnum = 0;
    move16();
    position = dminpos;
    move16();
    FOR ( i = length; i > 0; i--)
    {
        vout_fx[i-1] = quant_fx[position][i];
        move16();
        if (v_fx[i-1] <= 0)
        {
            vout_fx[i-1] = - quant_fx[position][i];
            move16();
        }
        position  = path_fx[position][i];
        move16();
        savedstates[i-1] = position;
        move32();

        /* calculate output pulses number & nz */
        pulsesnum = add(pulsesnum, abs_s(vout_fx[i-1]));/*quant[position][i]; */
        IF ( abs_s(vout_fx[i-1]) > 0 )
        {
            if ( nzpos == 0 )
            {
                *lasttrellislevel = i;
                move32();
            }

            nzpos = add(nzpos, 1);
        }
    }

    if ( pulsesout != 0 )
    {
        *pulsesout = pulsesnum;
        move16();
    }
    if ( nzposout != 0 )
    {
        *nzposout  = nzpos;
        move16();
    }

    return;
}

Word32 GetISCScale_fx( Word32 *quants_fx, Word16 size, Word32 bits_fx, Word16 *magn_fx, Word32 *qscale_fx, Word32 *surplus_fx, Word16 *pulses, Word32* savedstates, Word32 noTCQ,
                       Word16 *nzpout, Word16 *bcount, Word32 *abuffer, Word16 *mbuffer, Word32 *sbuffer)
{
    Word32 pos, terminate, trellislevel, st;
    Word32 lasttrellislevel;
    Word16 i, j, m_int, leftp, leftnz, nzpos, direction, pulsesnum, diff, pulsescurr, nzposcurr, counter;
    Word16 sign;
    Word16 flag_g1;
    Word32 t32;
    Word16 SafeExp;

    Word32 magnbits_fx = 0, tcqmagnbits_fx/*, surplus_fx, bits_fx*/;
    Word16 prob0_fx, prob1_fx, num, denum, quantum1_fx, quantum2_fx;
    Word32 dmin_fx, scale_fx32;
    Word32 actualt_fx;
    Word32 actualt_fx1 = 0;
    Word32 pt_fx, sxy_fx = 0, sy2_fx = 0;
    Word16 pn_fx, g_fx, scale_fx;
    Word16 tmp16, exp, exp1, exp2, Q_temp, hi, lo;
    Word32 m_fx, tmp32;
    Word32 /*quants_fx[TCQ_MAX_BAND_SIZE], */aquants_fx[TCQ_MAX_BAND_SIZE], dist_fx[TCQ_MAX_BAND_SIZE];
    Word16 exp_dist[TCQ_MAX_BAND_SIZE];
    Word16 /*magn_fx[TCQ_MAX_BAND_SIZE], */t_fx;
    Word16 Qscale;
    Word16 Qsxy=4;

    exp = 0;        /* to avoid compilation warnings */
    Qscale = 0;     /* to avoid compilation warnings */

    set32_fx( dist_fx, 0, size );
    set16_fx(exp_dist, 31, size );
    set32_fx( aquants_fx, 0, size );

    IF( bits_fx < 0 )
    {
        pulsesnum =  0;
        move16();

        IF( surplus_fx != 0 )
        {
            /* *surplus_fx += bits_fx; */
            *surplus_fx = L_add(*surplus_fx, bits_fx);
            move32();
        }
    }
    ELSE
    {
        pulsesnum = GetScale_fx(size, bits_fx, surplus_fx);
    }
    *nzpout = 0;
    move16();

    if ( pulses != 0 )
    {
        *pulses = pulsesnum;
        move16();
    }

    IF ( pulsesnum > 0 )
    {
        /* Initial quantization */
        m_fx = L_deposit_l(0);
        FOR (i = 0; i < size; i++)
        {
            aquants_fx[i] = L_abs(quants_fx[i]);/*Q12 */  move32();
            m_fx = L_add(m_fx, L_shr( aquants_fx[i], 2) );/*Q12 - 2*/
        }

        IF (m_fx == 0)
        {
            scale_fx = 0;
            move16();
            FOR (i = 0; i < size; i++)
            {
                magn_fx[i] = 0;
                move16();
            }
            t_fx = 0;
            move16();
        }
        ELSE
        {
            exp1 = sub(norm_s(pulsesnum), 1);
            exp2 = norm_l(m_fx);
            scale_fx = div_s(shl(pulsesnum, exp1), extract_h(L_shl(m_fx, exp2)));/*15 + exp1 - (exp2 + 12 - 16) */

            exp = 15 + exp1 - (exp2 + 12 - 16) + 2;
            move16();
            t_fx = 0;
            move16();
            FOR (i = 0; i < size; i++)
            {
                tmp32 = Mult_32_16(aquants_fx[i], scale_fx);/*12 + exp - 15 */
                tmp32 = L_shl(tmp32, sub(16, 12 + exp - 15));/*16 */
                magn_fx[i] = extract_h(L_add(32768, tmp32));
                t_fx = add(t_fx, magn_fx[i]);
            }
        }

        /* Pulses redistribution */
        WHILE (sub(t_fx, pulsesnum) != 0 )
        {
            pn_fx = 0;
            move16();
            pt_fx = L_deposit_l(0);

            nzpos = 0;
            move16();
            FOR (i = 0; i < size; i++)
            {
                IF (magn_fx[i] > 0)
                {
                    pn_fx = add(pn_fx, magn_fx[i]);/*0 */
                    pt_fx = L_add(pt_fx, L_shr( aquants_fx[i], 2) );/*12 */
                }
            }

            direction = -1;
            move16();
            if ( sub(pulsesnum, t_fx) > 0 )
            {
                direction = 1;
                move16();
            }

            /* new alg */
            {
                FOR ( i = 0; i < size; i++)
                {
                    sxy_fx = L_add(sxy_fx, L_shl(Mult_32_16(aquants_fx[i], magn_fx[i]), Qsxy+3));     /* 12+0-15 +9 -> 6 */
                    sy2_fx = L_add(sy2_fx, L_mult0(magn_fx[i], magn_fx[i]));/*0 */
                }
                Q_temp = 32;
                move16();
                FOR ( i = 0; i < size; i++)
                {
                    IF (magn_fx[i] > 0)
                    {
                        tmp16 = add(pn_fx, direction);
                        tmp32 = L_add(pt_fx, 0);
                    }
                    ELSE
                    {
                        tmp16 = add(pn_fx, direction);
                        tmp32 = L_add(pt_fx, L_shr( aquants_fx[i], 2) );
                    }

                    IF (tmp32 == 0)
                    {
                        g_fx = 0;
                        move16();
                    }
                    ELSE
                    {
                        exp1 = sub(norm_l(tmp32), 1);
                        exp2 = norm_s(tmp16);
                        tmp32 = L_shl(tmp32, exp1);
                        IF (tmp16 == 0)
                        {
                            tmp16 = 1;
                            move16();
                            exp2 = 16;
                            move16();
                        }
                        ELSE
                        {
                            tmp16 = shl(tmp16, exp2);
                        }
                        g_fx = div_s(extract_h(tmp32), tmp16);/*15 + 12 + exp1 - 16 - exp2; */
                        exp = 15 + 12 + exp1 - 16 - exp2 - 2;
                        move16();
                    }

                    IF (g_fx == 0)
                    {
                        dist_fx[i] = L_deposit_l(0);
                    }
                    ELSE
                    {
                        IF (direction > 0)
                        {
                            tmp32 = L_add(sxy_fx, L_shr(aquants_fx[i], 12-Qsxy));/*Qsxy */
                            t32 = L_add( sy2_fx, L_add( 1, L_deposit_l( shl(magn_fx[i], 1) ) ) );

                            IF( sub(norm_l(t32), 15) < 0 )
                            {
                                SafeExp = sub(16, norm_l(t32));
                                tmp16 = extract_l( L_shr( t32, SafeExp) );
                            }
                            ELSE
                            {
                                SafeExp = 0;
                                move16();
                                tmp16 = extract_l( t32 );
                            }
                        }
                        ELSE
                        {
                            tmp32 = L_sub(sxy_fx, L_shr(aquants_fx[i], 12-Qsxy));/*Qsxy */
                            t32 = L_add( sy2_fx, L_sub( 1, L_deposit_l( shl(magn_fx[i], 1) ) ) );
                            SafeExp = norm_l(t32);

                            IF( sub(norm_l(t32), 15) <= 0 )
                            {
                                SafeExp = sub(16, norm_l(t32));
                                tmp16 = extract_l( L_shr( t32, SafeExp) );
                            }
                            ELSE
                            {
                                SafeExp = 0;
                                move16();
                                tmp16 = extract_l( t32 );
                            }
                        }
                        tmp32 = L_shl(tmp32, 1-SafeExp); /* *2 */
                        tmp32 = L_sub(L_shl(L_mult0(g_fx, tmp16), sub(Qsxy, exp)), tmp32);/*Qsxy */
                        dist_fx[i] = Mult_32_16(tmp32, g_fx);/*Qsxy + exp - 15 */  move32();
                        exp_dist[i] = add(Qsxy-15, exp);
                        move16();
                        if (sub(exp_dist[i], Q_temp) < 0)
                        {
                            Q_temp = exp_dist[i];
                            move16();
                        }
                    }

                }
                FOR (i = 0; i < size; i++)
                {
                    dist_fx[i] = L_shr(dist_fx[i], sub(exp_dist[i], Q_temp));
                    move32();
                }
            }

            {
                /* find min */
                pos = L_deposit_l(0);
                dmin_fx = L_add(dist_fx[0], 0);
                FOR (i = 1; i < size; i++)
                {
                    IF (L_sub(dmin_fx, dist_fx[i]) > 0)
                    {
                        pos = L_deposit_l(i);
                        dmin_fx = L_add(dist_fx[i], 0);
                    }
                }
                /*IF (magn_fx[i] == 0 && direction < 0) */
                test();
                IF (magn_fx[pos] == 0 && direction < 0)
                {
                    pos = L_deposit_l(0);
                    FOR (i = 0; i < size; i++)
                    {
                        IF (magn_fx[i] != 0)
                        {
                            pos = L_deposit_l(i);
                            BREAK;
                        }
                    }
                    dmin_fx = L_add(dist_fx[i], 0);
                    FOR (; i < size; i++)
                    {
                        test();
                        IF (magn_fx[i] != 0 && L_sub(dmin_fx, dist_fx[i]) < 0)
                        {
                            pos = L_deposit_l(i);
                            dmin_fx = L_add(dist_fx[i], 0);
                        }
                    }
                }

                magn_fx[pos] = add(magn_fx[pos], direction);
                move16();
                t_fx = add(t_fx, direction);
            }
        }

        /* calculate actual nz positions */
        actualt_fx = L_deposit_l(0);
        nzpos = 0;
        move16();
        FOR (i = 0;  i < size; i++)
        {
            IF (magn_fx[i] > 0)
            {
                if (quants_fx[i] < 0)
                {
                    magn_fx[i] = negate(magn_fx[i]);
                    move16();
                }
                actualt_fx =  L_add(actualt_fx1, L_shr( aquants_fx[i], 2) );/*12 */
                nzpos++;
            }
            /*magn[i] = (float)magn_fx[i]; */
        }

        /* calculate scale */
        IF (actualt_fx == 0)
        {
            scale_fx32 = L_add(MAX_32, 0);
        }
        ELSE
        {
            exp1 = norm_l(actualt_fx);
            exp2 = sub(norm_l(pulsesnum), 1);
            lo = L_Extract_lc(L_shl(actualt_fx, exp1), &hi);
            scale_fx32 = Div_32(L_shl(pulsesnum, exp2), hi, lo);/*31 + exp2 - exp1 - 12 */
            Qscale = 31 + exp2 - exp1 - 12 + 2;
            move16();
        }

        *qscale_fx = scale_fx32;
        move32();
        *nzpout = nzpos;
        move16();


        test();
        test();
        IF ( (sub(nzpos, pulsesnum) != 0 && sub(nzpos, 1) > 0) && noTCQ == 0 )
        {
            counter = 0;
            move16();
            terminate = L_deposit_l(1);
            DO
            {
                counter++;
                move16();

                /*TCQnew( quants, scale, size, magn, pulsesnum, &pulsescurr, &nzposcurr, savedstates, &lasttrellislevel, terminate); */
                TCQnew_fx(quants_fx, scale_fx32, Qscale,
                size, magn_fx, pulsesnum, &pulsescurr, &nzposcurr, savedstates, &lasttrellislevel, terminate);
                IF (sub(pulsesnum, pulsescurr) > 0 )
                {
                    Word32 L_tmp;
                    L_tmp = L_add(scale_fx32, Mult_32_16(scale_fx32, 3277));
                    IF(Overflow == 0)
                    {
                        scale_fx32 = L_add(L_tmp, 0);
                    }
                    ELSE
                    {
                        L_tmp = L_shr(scale_fx32, 1);
                        scale_fx32 = L_add(L_tmp, Mult_32_16(L_tmp, 3277));
                        Qscale = sub(Qscale, 1);
                        Overflow = 0;
                    }
                }

                if(sub(pulsesnum, pulsescurr) < 0 )
                {
                    scale_fx32 = Mult_32_16(scale_fx32, 29491);
                }
                if(L_sub(counter, 3) > 0)
                {
                    terminate = L_deposit_l(1);
                }

            }
            WHILE (sub(pulsesnum, pulsescurr) != 0 && counter < 0 );
            /* You can use this code to check #pulses problem */
            /*IF (sub(pulsesnum, pulsescurr) != 0)
            {
            }*/

            IF (sub(pulsesnum, pulsescurr) > 0)
            {
                diff = sub(pulsesnum, pulsescurr);

                FOR ( i = size-1; i >=0; i--)
                {
                    IF ( abs(magn_fx[i]) > 0 )
                    {
                        sign = (magn_fx[i]>0)?(1) : (-1);
                        move16();
                        /*magn_fx[i] = sign * abs_s(add(magn_fx[i], diff));  move32(); */
                        IF (sign > 0)
                        {
                            magn_fx[i] = abs_s(add(magn_fx[i], diff));
                            move16();
                        }
                        ELSE
                        {
                            magn_fx[i] = -abs_s(add(magn_fx[i], diff));
                            move16();
                        }

                        BREAK;
                    }
                }
            }
            ELSE IF(sub(pulsesnum, pulsescurr) < 0)
            {
                diff = sub(pulsescurr, pulsesnum);

                FOR ( i = size-1; i >=0; i--)
                {
                    IF (diff <= 0)
                    {
                        BREAK;
                    }

                    IF ( abs(magn_fx[i]) > 0 )
                    {
                        sign = (magn_fx[i]>0)?(1) : (-1);
                        move16();
                        m_int = abs_s(magn_fx[i]);

                        IF (sub(diff, m_int) < 0)
                        {
                            /*magn_fx[i] = sign * sub(abs_s(magn_fx[i]), diff);  move16(); */
                            IF (sign > 0)
                            {
                                magn_fx[i] = sub(abs_s(magn_fx[i]), diff);
                                move16();
                            }
                            ELSE
                            {
                                magn_fx[i] = - sub(abs_s(magn_fx[i]), diff);
                                move16();
                            }
                            BREAK;
                        }
                        ELSE
                        {
                            diff = sub(diff, m_int);
                            magn_fx[i] = 0;
                            move16();
                            nzposcurr = sub(nzposcurr, 1);
                        }
                    }
                }
            }

            pulsescurr = pulsesnum;
            move16();

            /* Magnitudes coding */
            {
                leftp  = pulsescurr;
                move16();/*pulsesnum; */
                leftnz = nzposcurr;
                move16(); /*nzpos; */
                trellislevel = L_deposit_l(0);

                FOR ( i = 0; i < size; i++)
                {
                    IF (sub(leftnz, 1) <= 0)
                    {
                        BREAK;
                    }

                    IF ( magn_fx[i] != 0 )
                    {
                        FOR ( j = 0; j < leftp; j++)
                        {
                            num = sub(leftnz, 1);
                            denum = sub(leftp, add(j, 1));
                            IF (sub(num, denum) >= 0)
                            {
                                prob1_fx = MAX_16;
                                move16();
                                prob0_fx = 0;
                                move16();
                            }
                            ELSE
                            {
                                exp1 = sub(norm_s(num), 1);
                                exp2 = norm_s(denum);
                                prob1_fx = div_s(shl(num, exp1), shl(denum, exp2));/*15 + exp1 - exp2 */
                                exp = 15 + exp1 - exp2;
                                move16();
                                prob1_fx = shl(prob1_fx, sub(15, exp));
                                prob0_fx = sub(MAX_16, prob1_fx);
                            }

                            st = L_add(savedstates[ trellislevel ], 0);
                            quantum1_fx = quantize_fx(shl(add(j, 1), 10), ddec_fx[st][0]);
                            quantum2_fx = quantize_fx(shl(add(j, 1), 10), ddec_fx[st][1]);

                            test();
                            IF(sub(quantum1_fx, add(j, 1)) != 0 && sub(quantum2_fx, add(j, 1)) != 0 )
                            {
                                /* this magnitude is not possible so set probabilities */
                                prob0_fx = MAX_16;
                                move16();
                                prob1_fx = 0;
                                move16();
                            }

                            IF (sub(j, sub(abs_s(magn_fx[i]), 1)) < 0 )
                            {
                                exp1 = norm_s(prob0_fx);
                                tmp32 = L_deposit_h(shl(prob0_fx, exp1));/*exp1 + 15 + 16 */
                                tmp16 = Log2_norm_lc(tmp32);/*15 */
                                magnbits_fx = L_sub(magnbits_fx, L_sub(tmp16, L_shl(add(exp1, 1), 15)));/*15 */
                            }
                            ELSE
                            {
                                exp1 = norm_s(prob1_fx);
                                tmp32 = L_deposit_h(shl(prob1_fx, exp1));/*exp1 + 15 + 16 */
                                tmp16 = Log2_norm_lc(tmp32);/*15 */
                                magnbits_fx = L_sub(magnbits_fx, L_sub(tmp16, L_shl(add(exp1, 1), 15)));/*15 */
                                BREAK;
                            }
                        }

                        leftnz--;
                        move16();
                        leftp = sub(leftp, abs_s(magn_fx[i]));
                    }

                    trellislevel = L_add(trellislevel, 1);
                }

                /* Update actual occurred surplus */
                tcqmagnbits_fx = L_sub(L_sub(table_logcum_fx[pulsescurr], table_logcum_fx[nzposcurr]), table_logcum_fx[pulsescurr - (nzposcurr - 1)]);
                *surplus_fx = L_add(*surplus_fx, L_sub(tcqmagnbits_fx, L_shl(magnbits_fx, 1)));

                *nzpout = nzposcurr;
                move16();
            }/*magnitude coding */
        }/*TCQ */

        IF (actualt_fx == 0)
        {
            scale_fx32 = L_add(MAX_32, 0);
        }
        ELSE
        {
            exp1 = norm_l(actualt_fx);
            exp2 = sub(norm_l(pulsesnum), 1);
            lo = L_Extract_lc(L_shl(actualt_fx, exp1), &hi);
            scale_fx32 = Div_32(L_shl(pulsesnum, exp2), hi, lo);/*31 + exp2 - exp1 - 12 */
            Qscale = 31 + exp2 - exp1 - 12;
            move16();
        }
        test();
        IF ( *nzpout > 1 && bcount != 0 )
        {
            flag_g1 = 0;
            move16();

            FOR ( i = 0; i < size; i++)
            {
                if( abs_s(magn_fx[i]) > 1 )
                {
                    flag_g1 = 1;
                    move16();
                }
            }
            /* prepare vector for TCQ */
            FOR ( i = 0; i < size; i++)
            {
                test();
                IF (flag_g1 == 0 || sub(*bcount, 2*TCQ_AMP) >= 0)
                {
                    BREAK;
                }

                IF ( abs_s(magn_fx[i]) > 0 )
                {
                    abuffer[*bcount] = quants_fx[i];
                    move32();
                    mbuffer[*bcount] = magn_fx[i];
                    move16();
                    /*sbuffer[*bcount] = scale_fx32;*/
                    exp = norm_s(pulsesnum);
                    tmp16 = div_l( actualt_fx, shl( pulsesnum, exp-1) );
                    tmp32 = L_shl( L_deposit_l(tmp16), exp);
                    /*sbuffer[*bcount] = 1/((float)tmp32/pow(2.0, 12));*/
                    sbuffer[*bcount] = tmp32;/*Q12*/  move32();
                    /*sbuffer[*bcount] = (float)scale_fx32 / pow(2.0, Qscale);*/

                    (*bcount)++;

                }
            }
        }
        *qscale_fx = scale_fx32;
        move32();
    }

    return L_shl(magnbits_fx, 1);
}

void TCQLSB_fx(Word16 bcount, Word32 *abuffer_fx, Word16 *mbuffer_fx, Word32 *sbuffer_fx, Word16 *dpath)
{
    Word16 i, st, dminpos, position;
    Word16 q_fx = 6554;/*Q15*/
    Word32 dmin_fx, curdist1_fx, curdist2_fx, newdist1_fx, newdist2_fx;
    Word16 path[STATES_LSB][TCQ_LSB_SIZE];
    Word16 quant[STATES_LSB][TCQ_LSB_SIZE];
    Word16 dquant[STATES_LSB][TCQ_LSB_SIZE];
    Word16 qout[TCQ_LSB_SIZE];
    Word32 s1_fx, s2_fx, a1_fx, a2_fx;
    Word16 s1_fx16, s2_fx16;
    Word16 q1_fx, q2_fx, sign1_fx, sign2_fx;
    Word16 dbuffer_fx[MAX_PULSES];

    Word32 tmp1, tmp2;
    Word16 exp1, exp2;
    Word32 metric_fx[STATES_LSB][TCQ_LSB_SIZE];
    Word32 MaxPath;

    set32_fx(*metric_fx, 0, STATES_LSB*TCQ_LSB_SIZE);
    set16_fx( *path, 0, STATES_LSB*TCQ_LSB_SIZE );
    set16_fx( *quant, 0, STATES_LSB*TCQ_LSB_SIZE );
    set16_fx( *dquant, 0, STATES_LSB*TCQ_LSB_SIZE );
    set16_fx( qout, 0, TCQ_LSB_SIZE );

    metric_fx[1][0] = MAX_32>>2;
    move32();
    metric_fx[2][0] = MAX_32>>2;
    move32();
    metric_fx[3][0] = MAX_32>>2;
    move32();

    FOR( i = 0; i < 2*TCQ_AMP; i+=2 )
    {
        q1_fx = mbuffer_fx[i];
        move16();
        q2_fx = mbuffer_fx[i + 1];
        move16();

        s1_fx= L_add(sbuffer_fx[i], 0);  /*12*/
        s2_fx = L_add(sbuffer_fx[i + 1], 0);  /*12*/
        exp1 = norm_l(s1_fx);
        exp2 = norm_l(s2_fx);
        s1_fx16 = extract_h(L_shl(s1_fx, exp1));/*12 + exp1 - 16*/
        s2_fx16 = extract_h(L_shl(s2_fx, exp2));/*12 + exp2 - 16*/
        exp1 = 12 + exp1 - 16;
        move16();
        exp2 = 12 + exp2 - 16;
        move16();

        a1_fx = L_add(abuffer_fx[i], 0);
        a2_fx = L_add(abuffer_fx[i + 1], 0);

        MaxPath = L_add(MAX_32, 0);

        /* cycle over conditions */
        FOR ( st = 0; st < 4; st++)
        {
            curdist1_fx = L_add(metric_fx[ step_LSB_fx[st][0] ][i/2], 0);
            curdist2_fx = L_add(metric_fx[ step_LSB_fx[st][1] ][i/2], 0);

            /* step 1 */
            /*sign1_fx = (denc_LSB[st][0] & 0x1)?(q_fx):(-q_fx);*/
            IF (s_and(denc_LSB_fx[st][0], 0x1))
            {
                sign1_fx = q_fx;
                move16();
            }
            ELSE
            {
                sign1_fx = negate(q_fx);
            }
            /*sign2_fx = (denc_LSB[st][0] & 0x2)?(q_fx):(-q_fx);*/
            IF (s_and(denc_LSB_fx[st][0], 0x2))
            {
                sign2_fx = q_fx;
                move16();
            }
            ELSE
            {
                sign2_fx = negate(q_fx);
            }
            tmp1 = L_sub(a1_fx, L_shl(Mult_32_16(L_add(L_shl(q1_fx, 15), sign1_fx), s1_fx16), sub(12, exp1)));/*12*/
            tmp2 = L_sub(a2_fx, L_shl(Mult_32_16(L_add(L_shl(q2_fx, 15), sign2_fx), s2_fx16), sub(12, exp2)));/*12*/
            newdist1_fx = L_add(Mult_32_32(tmp1, tmp1), Mult_32_32(tmp2, tmp2)); /* -7 */

            /* step 2 */
            /*sign1_fx = (denc_LSB[st][1] & 0x1)?(q_fx):(-q_fx);*/
            IF (s_and(denc_LSB_fx[st][1], 0x1))
            {
                sign1_fx = q_fx;
                move16();
            }
            ELSE
            {
                sign1_fx = negate(q_fx);
            }
            /*sign2_fx = (denc_LSB[st][1] & 0x2)?(q_fx):(-q_fx);*/
            IF (s_and(denc_LSB_fx[st][1], 0x2))
            {
                sign2_fx = q_fx;
                move16();
            }
            ELSE
            {
                sign2_fx = negate(q_fx);
            }
            tmp1 = L_sub(a1_fx, L_shl(Mult_32_16(L_add(L_shl(q1_fx, 15), sign1_fx), s1_fx16), sub(12, exp1)));/*12*/
            tmp2 = L_sub(a2_fx, L_shl(Mult_32_16(L_add(L_shl(q2_fx, 15), sign2_fx), s2_fx16), sub(12, exp2)));/*12*/
            newdist2_fx = L_add(Mult_32_32(tmp1, tmp1), Mult_32_32(tmp2, tmp2));/*-7*/

            /* decision */
            IF ( L_sub(L_add(curdist1_fx, newdist1_fx), L_add(curdist2_fx, newdist2_fx)) < 0 )
            {
                path[st][i/2+1] = step_LSB_fx[st][0];
                move16();
                metric_fx[st][i/2+1] = L_add(curdist1_fx, newdist1_fx);
                move32();
                quant[st][i/2+1] = 0;
                move16();
                dquant[st][i/2+1] = dqnt_LSB_fx[ step_LSB_fx[st][0] ][st];
                move16();
            }
            ELSE
            {
                path[st][i/2+1] = step_LSB_fx[st][1];
                move16();
                metric_fx[st][i/2+1] = L_add(curdist2_fx, newdist2_fx);
                move32();
                quant[st][i/2+1] = 1;
                move16();
                dquant[st][i/2+1] = dqnt_LSB_fx[ step_LSB_fx[st][0] ][st];
                move16();
            }

            if( L_sub( MaxPath, metric_fx[st][i/2+1]) > 0 )
            {
                MaxPath = L_add(metric_fx[st][i/2+1], 0);
            }
        }
        /* Metric renormalization to prevent overflow */
        FOR ( st = 0; st < 4; st++)
        {
            metric_fx[st][i/2+1] = L_sub( metric_fx[st][i/2+1], MaxPath );
            move32();
        }
    }

    /* Find path with minimal metric */
    dminpos = 0;
    move16();
    dmin_fx = L_add(metric_fx[ dminpos][ i/2], 0);
    FOR ( st = 1; st < 4; st++)
    {
        IF ( L_sub(dmin_fx, metric_fx[ st][ i/2]) > 0 )
        {
            dmin_fx = L_add(metric_fx[ st][ i/2], 0);
            dminpos = st;
            move16();
        }
    }

    /* Trace back to get output */
    position = dminpos;
    move16();

    FOR ( ; i >= 0; i-=2)
    {
        qout[i/2] = quant[position][ i/2+1 ];
        move16();
        dpath[i/2] = dquant[position][ i/2+1 ];
        move16();

        IF (s_and(denc_LSB_fx[position][qout[i/2]], 0x1))
        {
            dbuffer_fx[i] = 1;
            move16();
        }
        ELSE
        {
            dbuffer_fx[i] = -1;
            move16();
        }
        IF (s_and(denc_LSB_fx[position][qout[i/2]], 0x2))
        {
            dbuffer_fx[i + 1] = 1;
            move16();
        }
        ELSE
        {
            dbuffer_fx[i + 1] = -1;
            move16();
        }

        position  = path[position][i/2+1];
        move16();
    }

    /* add decoded sequence to quanta */
    FOR ( i = 0; i < bcount; i++ )
    {
        mbuffer_fx[i] = add(add(mbuffer_fx[i], shl(mbuffer_fx[i], 2)), dbuffer_fx[i]);
    }

    return;
}
void TCQLSBdec_fx( Word16 *dpath, Word16 *mbuffer, Word16 bcount )
{
    /*float q = QTCQ;*/
    Word16 q = 1;/*x5*/
    Word16 i, tmp, state = 0;

    tmp = shr(bcount, 1);
    FOR ( i = 0; i < tmp; i++)
    {
        IF (s_and(ddec_LSB_fx[state][dpath[i]], 0x1))
        {
            mbuffer[2*i] = q;
            move16();
        }
        ELSE
        {
            mbuffer[2*i] = negate(q);
        }

        IF (s_and(ddec_LSB_fx[state][dpath[i]], 0x2))
        {
            mbuffer[2*i + 1] = q;
            move16();
        }
        ELSE
        {
            mbuffer[2*i + 1] = negate(q);
        }

        state  = dstep_LSB_fx[state][dpath[i]];
        move16();
    }

    return;
}
void RestoreTCQ_fx( Word16 * magn, Word16 size, Word16 *bcount, Word16 *mbuffer)
{
    Word16 i, nzpos = 0, flag_g1 = 0;

    /* calculate actual nz positions */
    nzpos = 0;
    move16();
    FOR ( i = 0; i < size; i++)
    {
        IF ( magn[i] != 0 )
        {
            nzpos = add(nzpos, 1);
            if ( sub(abs_s(magn[i]), 5) > 0 )
            {
                flag_g1 = 1;
                move16();
            }
        }
    }

    IF ( sub(nzpos, 1) > 0)
    {
        FOR( i = 0; i < size; i++)
        {
            test();
            IF (flag_g1 == 0 || sub(*bcount, 2*TCQ_AMP) >= 0)
            {
                BREAK;
            }

            IF( abs_s(magn[i]) > 0 )
            {
                magn[i] = mbuffer[*bcount];
                move16();
                *bcount = add(*bcount, 1);
            }
        }
    }

    return;
}
void RestoreTCQdec_fx( Word16 * magn, Word16 size, Word16 *bcount, Word16 *mbuffer)
{
    Word16 i, nzpos = 0, flag_g1 = 0;

    /* calculate actual nz positions */
    nzpos = 0;
    move16();
    FOR ( i = 0; i < size; i++)
    {
        IF( magn[i] != 0 )
        {
            nzpos = add(nzpos, 1);
            if ( sub(abs_s(magn[i]), 1) > 0 )
            {
                flag_g1 = 1;
                move16();
            }
            magn[i] = extract_l(L_mult0(magn[i], 5));
        }
    }

    IF( sub(nzpos, 1) > 0)
    {
        FOR ( i = 0; i < size; i++)
        {
            test();
            IF (!(flag_g1 && sub(*bcount, 2*TCQ_AMP) < 0))
            {
                BREAK;
            }

            IF ( magn[i] != 0 )
            {
                mbuffer[*bcount] = add(magn[i], mbuffer[*bcount]);
                move16();
                magn[i] = mbuffer[*bcount];
                move16();
                *bcount = add(*bcount, 1);
            }
        }
    }

    return;
}

void InitLSBTCQ_fx(Word16 *bcount)
{
    *bcount = 0;
    move16();

    return;
}
void SaveTCQdata_fx( PARCODEC_FX arInst, Word16 *dpath, Word16 bcount)
{
    Word16 i;
    FOR ( i = 0; i < bcount; i++)
    {
        ar_encode_uniform_fx( arInst, dpath[i], 1);
    }

    return;
}
void LoadTCQdata_fx( PARCODEC_FX arInst, Word16 *dpath, Word16 bcount)
{
    Word32 i;
    FOR ( i = 0; i < bcount; i++)
    {
        dpath[i] = ar_decode_fx( arInst, uniform_model_fx );
    }

    return;
}


Word32 encode_position_ari_fx(PARCODEC_FX parenc, Word16* quants, Word16 size, Word32* est_bits_frame_fx)
{
    Word16 i, tmp;
    Word16 nz = 0, pulses = 0;
    Word16 prob[TCQ_MAX_BAND_SIZE];
    Word16 model_num_nz[TCQ_MAX_BAND_SIZE];
    Word16 *cur_quants = quants;
    Word16 integer, frac;
    Word32 /*est_bits_frame_fx, */btcq_fx = 0, bits_fx = 0, pnzp_fx;

    Word32 cp, scp, fxone, fxp1;
    Word16 pos;

    fxone = 32768;
    move32();
    fxp1  = 512*32768;
    move32();



    set16_fx( prob, 0, TCQ_MAX_BAND_SIZE );
    set16_fx( model_num_nz, 0, TCQ_MAX_BAND_SIZE );

    FOR (i = 0; i < size; i ++)
    {
        pulses = add(pulses, abs_s(cur_quants[i]));
        if (cur_quants[i] != 0)
        {
            nz = add(nz, 1);
        }
    }

    btcq_fx = GetBitsFromPulses_fx(pulses, size);
    /* Estimate TCQ bits */
    bits_fx = L_sub(table_logcum_fx[size + 1], L_add(table_logcum_fx[nz + 1], table_logcum_fx[size - nz + 1]));
    bits_fx = L_add(bits_fx, L_sub(btcq_fx, L_sub(table_logcum_fx[size + 1], L_add(table_logcum_fx[nz + 1], table_logcum_fx[size - nz + 1]))));
    bits_fx = L_sub(bits_fx, L_sub(table_logcum_fx[pulses], L_add(table_logcum_fx[nz], table_logcum_fx[pulses - (nz - 1)])));
    bits_fx = L_sub(bits_fx, nz);
    *est_bits_frame_fx = L_add(*est_bits_frame_fx, bits_fx);

    /*caculate the #nz probability */
    tmp = s_min(pulses, size);
    FOR (i = 0; i < tmp; i++)
    {
        pnzp_fx = L_sub(L_deposit_h(add(i, 1)), btcq_fx);
        pnzp_fx = L_add(pnzp_fx, L_add(L_sub(table_logcum_fx[size + 1], L_add(table_logcum_fx[i + 2], table_logcum_fx[size - i])),
                                       L_sub(table_logcum_fx[pulses], L_add(table_logcum_fx[i + 1], table_logcum_fx[pulses - i]))));
        pnzp_fx = L_add(pnzp_fx, 917498);/*16 */
        integer = extract_h(pnzp_fx);
        frac = extract_l(L_shr(L_sub(pnzp_fx, L_deposit_h(integer)), 1));/*15 */
        prob[i] = extract_h(L_shl(Pow2(integer, frac), 16));/*0 */

        /*zero probability will incur problems in ar_make_model() */
        if (prob[i] == 0)
        {
            prob[i] = 1;
            move16();
        }
    }

    ar_make_model_fx(prob, model_num_nz, min(pulses, size));

    IF (sub(nz, 1) > 0)
    {
        ar_encode_fx(parenc, model_num_nz, nz - 1);/*encode #nz */
        scp = L_add(fxp1, 0);
        pos = 0;
        move16();
        FOR( i = 0; i < size; i++)
        {
            IF (nz <= 0)
            {
                BREAK;
            }

            IF( nz == (size - i) )
            {
                cp = L_deposit_l(0);
            }
            ELSE
            {
                cp = L_sub( fxone, div_l( L_deposit_h(nz), (size - i)) );
            }
            scp = Mult_32_16( scp, extract_l(cp) );
            model_num_nz[pos+1] = round_fx( L_shl( scp, 6) );

            test();
            test();
            IF( (model_num_nz[pos+1] == 0 && scp > 0) || model_num_nz[pos] == model_num_nz[pos+1] )
            {
                model_num_nz[pos+1] = 0;
                move16();
                ar_encode_fx( parenc, model_num_nz, pos );
                i--;
                move16();
                scp = L_add(fxp1, 0);
                pos = 0;
                move16();
                CONTINUE;
            }

            IF( cur_quants[i] != 0 )
            {
                ar_encode_fx( parenc, model_num_nz, pos );
                pos = 0;
                move16();
                scp = L_add(fxp1, 0);
                nz--;
                move16();
            }
            ELSE
            {
                pos++;
                move16();
            }
        }
    }
    ELSE IF (sub(nz, 1) == 0)
    {
        IF (sub(pulses, 1) > 0)
        {
            /*temp -= log2_f((float)(model_num_nz[nz-1] - model_num_nz[nz]) / MAX_AR_FREQ); */
            ar_encode_fx(parenc, model_num_nz, 0);/*encode #nz */
        }
        pos = 0;
        move16();
        FOR( i = 0; i < size; i++)
        {
            model_num_nz[i+1] = round_fx( L_shr( L_deposit_h( div_l( L_deposit_h(size - i - 1), size)), 1) );

            if( cur_quants[i] != 0 )
            {
                pos = i;
                move16();
            }
        }
        ar_encode_fx( parenc, model_num_nz, pos ); /* encode pos */
    }
    return bits_fx;
}
Word32 encode_magnitude_usq_fx(ARCODEC_FX* parenc, Word16* magn_fx, Word16 size, Word16 npulses, Word16 nzpos, Word32* est_frame_bits_fx)
{
    Word16 i, j, k, tmp, magnp, magnzp;
    Word16 magn_position[MAX_PULSES];
    Word32 /*est_frame_bits_fx, */bits_fx;
    Word16 pos, model_m[MAX_PULSES + 2];
    Word32 fxone, fxp1, cp, scp;

    fxone = 32768;
    move32();
    fxp1  = 512*32768;
    move32();


    /*estimate fac bits */
    bits_fx = L_sub(table_logcum_fx[npulses], L_add(table_logcum_fx[nzpos], table_logcum_fx[npulses - nzpos + 1]));
    *est_frame_bits_fx = L_add(*est_frame_bits_fx, bits_fx);

    test();
    IF (sub(npulses, nzpos) == 0 || sub(nzpos, 1) == 0)
    {
        return bits_fx;
    }
    magnp = sub(npulses, 1);
    magnzp = sub(nzpos, 1);

    /*generate the binary sequences of magnitudes */
    k = 0;
    move16();
    FOR (i = 0; i < size; i++)
    {
        IF (magn_fx[i] != 0)
        {
            tmp = sub(abs_s(magn_fx[i]), 1);
            FOR (j = 0; j < tmp; j++)
            {
                magn_position[k++] = 0;
                move16();
            }
            magn_position[k++] = 1;
            move16();
        }
    }

    set16_fx( model_m, 0, MAX_PULSES + 2);
    scp = L_add(fxp1, 0);
    model_m[0] = MAX_AR_FREQ;
    move16();
    pos = 0;
    move16();
    tmp = sub(npulses, 1);
    FOR( i = 0; i < tmp; i++ )
    {
        IF (magnzp <= 0)
        {
            BREAK;
        }

        IF( magnzp == magnp )
        {
            cp = L_deposit_l(0);
        }
        ELSE
        {
            cp = L_sub( fxone, div_l( L_deposit_h(magnzp), magnp) );
        }
        scp = Mult_32_16( scp, extract_l(cp) );
        model_m[pos+1] = round_fx( L_shl( scp, 6) );

        test();
        test();
        IF( (model_m[pos+1] == 0 && scp > 0) || model_m[pos] == model_m[pos+1] )
        {
            model_m[pos+1] = 0;
            move16();

            ar_encode_fx( parenc, model_m, pos );
            pos = 0;
            move16();
            i--;
            move16();
            scp = L_add(fxp1, 0);
            CONTINUE;
        }

        IF( magn_position[i] != 0 )
        {
            ar_encode_fx( parenc, model_m, pos );
            pos = 0;
            move16();
            magnzp--;
            move16();
            scp = L_add(fxp1, 0);
        }
        ELSE
        {
            pos++;
            move16();
        }

        magnp--;
        move16();
    }
    return bits_fx;
}
Word32 encode_magnitude_tcq_fx(ARCODEC_FX* parenc, Word16* magn_fx, Word16 size, Word16 npulses, Word16 nzpos, Word32* savedstates, Word32* est_frame_bits_fx)
{
    Word32 tcq_bits_fx, bits_fx/*, est_frame_bits_fx*/;
    Word16 prob0_fx, prob1_fx, num, denum, quantum1_fx, quantum2_fx;
    Word16 exp, exp1, exp2;

    Word16 i, j;
    Word32 st;
    Word16 magn_mode[3] = {MAX_AR_FREQ, 0, 0};

    Word16 leftp  = npulses;/*pulsesnum; */
    Word16 leftnz = nzpos;/*nzpos; */
    move16();
    move16();

    bits_fx = L_deposit_l(0);
    tcq_bits_fx = L_sub(table_logcum_fx[npulses], L_add(table_logcum_fx[nzpos], table_logcum_fx[npulses - (nzpos - 1)]));
    *est_frame_bits_fx = L_add(*est_frame_bits_fx, tcq_bits_fx);

    test();
    IF (sub(nzpos, npulses) == 0 || sub(nzpos, 1) == 0)
    {
        return bits_fx;
    }

    st = L_deposit_l(0);
    FOR ( i = 0; i < size; i++)
    {
        IF (sub(leftnz, 1) <= 0)
        {
            BREAK;
        }

        st = L_add(savedstates[i], 0);
        IF (magn_fx[i] != 0)
        {
            FOR ( j = 0; j < leftp; j++)
            {
                /*calculate the two path probs point to next two states */
                num = sub(leftnz, 1);
                denum = sub(leftp, add(j, 0x1));
                IF (sub(num, denum) >= 0)
                {
                    prob1_fx = MAX_16;
                    move16();
                    prob0_fx = 0;
                    move16();
                }
                ELSE
                {
                    exp1 = sub(norm_s(num), 1);
                    exp2 = norm_s(denum);
                    prob1_fx = div_s(shl(num, exp1), shl(denum, exp2));/*15 + exp1 - exp2 */
                    exp = 15 + exp1 - exp2;
                    move16();
                    prob1_fx = shl(prob1_fx, sub(15, exp));
                    prob0_fx = sub(MAX_16, prob1_fx);
                }

                quantum1_fx = quantize_fx(shl(add(j, 1), 10), ddec_fx[st][0]);
                quantum2_fx = quantize_fx(shl(add(j, 1), 10), ddec_fx[st][1]);

                test();
                IF (sub(quantum1_fx, add(j, 1)) != 0 && sub(quantum2_fx, add(j, 1)) != 0)
                {
                    prob0_fx = MAX_16;
                    move16();
                    prob1_fx = 0;
                    move16();
                }

                test();
                IF (sub(prob0_fx, MAX_16) == 0 || sub(prob1_fx, MAX_16) == 0)
                {
                    CONTINUE;
                }

                magn_mode[1] = mult(prob1_fx, MAX_AR_FREQ);
                IF (sub(j, sub(abs_s(magn_fx[i]), 1)) < 0)
                {
                    ar_encode_fx(parenc, magn_mode, 0);
                }
                ELSE
                {
                    IF (sub(leftp, j) > leftnz)
                    {
                        ar_encode_fx(parenc, magn_mode, 1);
                    }
                    BREAK;
                }
            }

            leftnz--;
            move16();
            leftp = sub(leftp, abs_s( magn_fx[i] ));
        }
    }

    return bits_fx;
}
Word32 encode_signs_fx(ARCODEC_FX* parenc, Word16* magn, Word16 size, Word16 npos, Word32* est_frame_bits_fx)
{
    Word32 i, sign;

    *est_frame_bits_fx = L_add(*est_frame_bits_fx, L_deposit_h(npos));
    FOR (i = 0; i < size; i++)
    {
        IF (magn[i] != 0)
        {
            sign = L_deposit_l(0);
            if (magn[i] > 0)
            {
                sign = L_deposit_l(1);
            }
            ar_encode_uniform_fx(parenc, sign, 1);
        }
    }

    return L_deposit_h(npos);
}
void decode_position_ari_fx(PARCODEC_FX pardec, Word16 size, Word16 npulses, Word16* nz, Word16* position)
{
    Word16 i, tmp, nzp;
    Word16 mode_num_nz[TCQ_MAX_BAND_SIZE];
    Word16 prob[TCQ_MAX_BAND_SIZE];

    Word32 btcq_fx, pnzp_fx;
    Word16 integer, frac;
    Word32 cp, scp, fxone, fxp1;
    Word16 stpos = 0, pos, ovrflag = 0, temppos, storepos;

    fxone = 32768;
    move32();
    fxp1  = 512*32768;
    move32();
    temppos = 0;
    move16();
    storepos = 0;
    move16();

    set16_fx( mode_num_nz, 0, TCQ_MAX_BAND_SIZE );
    set16_fx( prob, 0, TCQ_MAX_BAND_SIZE );

    FOR (i = 0; i < size; i++)
    {
        position[i] = 0;
        move16();
    }

    IF (L_sub(npulses, 1) > 0)
    {
        btcq_fx = GetBitsFromPulses_fx(npulses, size);
        tmp = s_min(npulses, size);
        FOR (i = 0; i < tmp; i++)
        {
            /*calculate the probability of #nz */

            pnzp_fx = L_sub(L_deposit_h(add(i, 1)), btcq_fx);
            pnzp_fx = L_add(pnzp_fx, L_add(L_sub(table_logcum_fx[size + 1], L_add(table_logcum_fx[i + 2], table_logcum_fx[size - i])),
                                           L_sub(table_logcum_fx[npulses], L_add(table_logcum_fx[i + 1], table_logcum_fx[npulses - i]))));
            pnzp_fx = L_add(pnzp_fx, 917498);/*16 */
            integer = extract_h(pnzp_fx);
            frac = extract_l(L_shr(L_sub(pnzp_fx, L_deposit_h(integer)), 1));/*15 */
            prob[i] = extract_h(L_shl(Pow2(integer, frac), 16));/*0 */

            /*set the minimum pro 1/MAX_AR_FREQ to avoid some unpredictable problem */
            if (prob[i] == 0)
            {
                prob[i] = 1;
                move16();
            }
        }

        ar_make_model_fx(prob, mode_num_nz, min(npulses, size));
        *nz = add(1, ar_decode_fx(pardec, mode_num_nz));/*get #nz */
        nzp = *nz;
        move16();
        IF( nzp == 1 )
        {
            mode_num_nz[0] = MAX_AR_FREQ;
            move16();
            FOR (i = 0; i < size; i ++)
            {
                mode_num_nz[i+1] = round_fx( L_shr( L_deposit_h( div_l( L_deposit_h(size - i - 1), size)), 1) );
            }

            position[ ar_decode_fx(pardec, mode_num_nz) ] = 1;
            move16();
        }
        ELSE
        {
            mode_num_nz[0] = MAX_AR_FREQ;
            move16();

            FOR( ; nzp > 0; nzp-- )
            {
                scp = L_add(fxp1, 0);
                temppos  = 0;
                move16();
                storepos = 0;
                move16();

                FOR( i = stpos; i < size; i++)
                {
                    ovrflag = 0;
                    move16();

                    IF( nzp == (size - i) )
                    {
                        cp = L_deposit_l(0);
                    }
                    ELSE
                    {
                        cp = L_sub( fxone, div_l( L_deposit_h(nzp), (size - i)) );
                    }
                    scp = Mult_32_16( scp, extract_l(cp) );
                    mode_num_nz[i+1-storepos-stpos] = round_fx( L_shl( scp, 6) );

                    test();
                    test();
                    IF( (mode_num_nz[i+1-storepos-stpos] == 0 && scp > 0) || mode_num_nz[i-storepos-stpos] == mode_num_nz[i+1-storepos-stpos] )
                    {
                        mode_num_nz[i+1-storepos-stpos] = 0;
                        move16();
                        ovrflag = 1;
                        move16();
                        temppos = ar_decode_fx(pardec, mode_num_nz);
                        move16();
                        storepos += temppos;
                        move16();
                        scp = L_add(fxp1, 0);

                        IF( temppos == i-stpos) /* esc transmitted */
                        {
                            i--;
                            move16();
                        }
                        ELSE
                        {
                            BREAK;
                        }
                    }
                }
                IF( !ovrflag )
                {
                    pos = ar_decode_fx(pardec, mode_num_nz) + storepos;
                    move16();
                }
                ELSE
                {
                    pos = storepos;
                    move16();
                }

                position[ stpos + pos] = 1;
                move16();
                stpos += pos + 1;
                move16();
            }
        }
    }
    ELSE IF (L_sub(npulses, 1) == 0)
    {
        *nz = npulses;
        move16();
        nzp = *nz;
        move16();
        mode_num_nz[0] = MAX_AR_FREQ;
        move16();
        FOR (i = 0; i < size; i ++)
        {
            mode_num_nz[i+1] = round_fx( L_shr( L_deposit_h( div_l( L_deposit_h(size - i - 1), size)), 1) );
        }

        position[ ar_decode_fx(pardec, mode_num_nz) ] = 1;
        move16();
    }
    ELSE
    {
        *nz = 0;
        move16();
    }

    return;
}
void decode_magnitude_usq_fx(ARCODEC_FX* pardec, Word16 size, Word16 npulses, Word16 nzpos, Word16* positions, Word16* out)
{
    Word16 i, magnp, magnzp;
    Word16 magns[TCQ_MAX_BAND_SIZE], magncout = 0;
    Word16 storemagn, ovrflag = 0, pos, tempmagn = 0, mmodel[MAX_PULSES+2];
    Word32 cp, scp, fxone, fxp1;

    fxone = 32768;
    move32();
    fxp1  = 512*32768;
    move32();

    set16_fx( magns, 1, TCQ_MAX_BAND_SIZE );
    IF (sub(nzpos, npulses) == 0)
    {
        FOR (i = 0; i < size; i++)
        {
            out[i] = positions[i];
            move16();
        }
        return;
    }
    ELSE IF (sub(nzpos, 1) == 0)
    {
        FOR (i = 0; i < size; i++)
        {
            IF ( positions[i] != 0 )
            {
                out[i] = npulses;
                move16();
                return;
            }
        }
    }

    magnzp = sub(nzpos, 1);
    magnp = sub(npulses, 1);

    magncout = 0;
    move16();

    set16_fx( out, 0, size );
    set16_fx( mmodel, 0, MAX_PULSES+2 );

    mmodel[0] = MAX_AR_FREQ;
    move16();
    magncout = 0;
    move16();
    FOR( pos = 0; pos < size; pos++)
    {
        scp = L_add(fxp1, 0);
        IF( positions[pos] != 0)
        {
            storemagn = 0;
            move16();

            FOR( i = 0; i < magnp; i++)
            {
                ovrflag = 0;
                move16();

                IF( magnzp == (magnp-i) )
                {
                    cp = L_deposit_l(0);
                }
                ELSE
                {
                    cp = L_sub( fxone, div_l( L_deposit_h(magnzp), magnp-i) );
                }

                IF( cp == fxone )
                {
                    BREAK;
                }

                scp = Mult_32_16( scp, extract_l(cp) );
                mmodel[i+1-storemagn] = round_fx( L_shl( scp, 6) );

                test();
                test();
                IF( (mmodel[i+1- storemagn] == 0 && scp > 0) || mmodel[i- storemagn] == mmodel[i+1- storemagn] )
                {
                    mmodel[i+1-storemagn] = 0;
                    move16();
                    /* read data */
                    tempmagn = ar_decode_fx( pardec, mmodel );
                    storemagn += tempmagn;
                    move16();

                    IF( tempmagn < i )
                    {
                        /* just magnitude */
                        ovrflag = 1;
                        move16();
                        BREAK;
                    }
                    ELSE
                    {
                        /* esc code */
                        scp = L_add(fxp1, 0);
                        i--;
                        move16();
                    }
                }
            }

            IF( ovrflag )
            {
                out[magncout] = storemagn + 1;
                move16();
            }
            ELSE
            {
                out[magncout] = ar_decode_fx( pardec, mmodel ) + storemagn + 1;
                move16();
            }
            magnp -= out[magncout];
            move16();
            magnzp--;
            move16();
            magncout++;
            move16();

            IF (magnzp == 0) /* last magnitude generation */
            {
                FOR( pos = add(pos,1); pos < size; pos++)
                {
                    IF( positions[pos] != 0)
                    {
                        out[magncout] = magnp + 1;
                        move16();
                        return;
                    }
                    ELSE
                    {
                        out[magncout] = 0;
                        move16();
                        magncout++;
                        move16();
                    }
                }
            }
            ELSE IF(magnzp == magnp) /* rest magnitudes generation */
            {
                FOR( pos = add(pos,1); pos < size; pos++)
                {
                    out[magncout] = positions[pos];
                    move16();
                    magncout++;
                    move16();
                }
                return;
            }
        }
        ELSE
        {
            out[magncout] = 0;
            move16();
            magncout++;
            move16();
        }
    }

    return;
}
void decode_mangitude_tcq_fx(ARCODEC_FX* pardec, Word16 size, Word16 npulses, Word16 nzpos, Word16* positions, Word16* out, Word32* surplus_fx)
{
    Word32 tcq_bits_fx, bits_fx/*, surplus_fx*/;
    Word16 prob0_fx, prob1_fx, num, denum, quantum1_fx, quantum2_fx;
    Word16 exp, exp1, exp2, tmp16;
    Word32 tmp32;

    Word16 i, j, symbol, st;
    Word16 leftp  = npulses;/*pulsesnum; */
    Word16 leftnz = nzpos; /*nzpos; */
    Word16 magn_mode[3] = {MAX_AR_FREQ, 0, 0};

    bits_fx = L_deposit_l(0);
    tcq_bits_fx = L_sub(table_logcum_fx[npulses], L_add(table_logcum_fx[nzpos], table_logcum_fx[npulses - (nzpos - 1)]));

    IF (sub(nzpos, npulses) == 0)
    {
        FOR (i = 0; i < size; i++)
        {
            out[i] = positions[i];
            move16();
        }

        return;
    }
    ELSE IF (sub(nzpos, 1) == 0)
    {
        FOR (i = 0; i < size; i++)
        {
            IF ( positions[i] != 0 )
            {
                out[i] = npulses;
                move16();
                return;
            }
        }
    }
    st = 0;
    move16();
    FOR (i = 0; i < size; i++)
    {
        IF (sub(leftnz, 1) <= 0)
        {
            BREAK;
        }

        out[i] = positions[i];
        IF (positions[i] != 0)
        {
            /*generate the trellis path */
            symbol = 0;
            move16();
            FOR (j = 0; j < leftp; j++)
            {
                num = sub(leftnz, 1);
                denum = sub(leftp, add(j, 1));
                IF (sub(num, denum) >= 0)
                {
                    prob1_fx = MAX_16;
                    move16();
                    prob0_fx = 0;
                    move16();
                }
                ELSE
                {
                    exp1 = sub(norm_s(num), 1);
                    exp2 = norm_s(denum);
                    prob1_fx = div_s(shl(num, exp1), shl(denum, exp2));/*15 + exp1 - exp2 */
                    exp = 15 + exp1 - exp2;
                    prob1_fx = shl(prob1_fx, sub(15, exp));
                    prob0_fx = sub(MAX_16, prob1_fx);
                }
                IF (L_sub(sub(leftp, j), leftnz) == 0)
                {
                    symbol = add(j, 1);
                    BREAK;
                }

                quantum1_fx = quantize_fx(shl(add(j, 1), 10), ddec_fx[st][0]);
                quantum2_fx = quantize_fx(shl(add(j, 1), 10), ddec_fx[st][1]);

                test();
                IF (sub(quantum1_fx, add(j, 1)) != 0 && sub(quantum2_fx, add(j, 1)) != 0)
                {
                    prob0_fx = MAX_16;
                    move16();
                    prob1_fx = 0;
                    move16();
                }

                test();
                IF (sub(prob0_fx, MAX_16) == 0 || sub(prob1_fx, MAX_16) == 0)
                {
                    symbol = add(j, 1);
                    CONTINUE;
                }

                /*magn_mode[1] = (short)(prob1 * MAX_AR_FREQ); */
                magn_mode[1] = mult(prob1_fx, MAX_AR_FREQ);

                IF (ar_decode_fx(pardec, magn_mode))
                {
                    exp1 = norm_s(prob1_fx);
                    tmp32 = L_deposit_h(shl(prob1_fx, exp1));/*exp1 + 15 + 16 */
                    tmp16 = Log2_norm_lc(tmp32);/*15 */
                    bits_fx = L_sub(bits_fx, L_sub(tmp16, L_shl(add(exp1, 1), 15)));/*15 */

                    symbol = add(j, 1);
                    BREAK;
                }
                ELSE
                {
                    exp1 = norm_s(prob0_fx);
                    tmp32 = L_deposit_h(shl(prob0_fx, exp1));/*exp1 + 15 + 16 */
                    tmp16 = Log2_norm_lc(tmp32);/*15 */
                    bits_fx = L_sub(bits_fx, L_sub(tmp16, L_shl(add(exp1, 1), 15)));/*15 */
                }
            }
            out[i] = symbol;
            move16();
            /*leftp -= symbol; */
            leftp = sub(leftp, symbol);
            leftnz = sub(leftnz, 1);
        }

        quantum1_fx = quantize_fx(out[i], ddec_fx[st][0]);
        quantum2_fx = quantize_fx(out[i], ddec_fx[st][1]);

        /*generate the next state */
        IF (sub(quantum1_fx, out[i]) == 0)
        {
            st = nextstate[st][0];
            move16();
        }
        ELSE
        {
            st = nextstate[st][1];
            move16();
        }
    }

    /*generate the magnitudes */
    FOR (; i < size; i++)
    {
        out[i] = 0;
        move16();
        IF (positions[i] != 0)
        {
            out[i] = add(sub(leftp, leftnz), 1);
        }

    }

    test();
    IF (sub(nzpos, npulses) != 0 && sub(nzpos, 1) > 0)
    {
        /*update the surplus */
        *surplus_fx = L_add(*surplus_fx, L_sub(tcq_bits_fx, L_shl(bits_fx, 1)));
    }

    return;
}

void decode_signs_fx(ARCODEC_FX* pardec, Word16 size, Word16* out)
{
    Word16 i;

    FOR ( i = 0; i < size; i++)
    {
        IF ( out[i] != 0  )
        {
            out[i] = ( ar_decode_fx( pardec, uniform_model_fx ) > 0) ? out[i] : -out[i];
            move16();
        }
    }

    return;
}

Word16 GetScale_fx( Word16 blen, Word32 bits_fx/*Q16*/, Word32 *surplus_fx/*Q16*/)
{
    Word16 pulses = MAX_PULSES, p_est, exp, exp1, exp2, magicnum;
    Word32 t, a, b, ab, estbits_fx = 0;

    magicnum = 24773;
    move16(); /*Q17: 0.188992013101951f; */

    t = L_shr( L_mult( magicnum, blen), 2);
    exp = norm_l(t);
    a = L_shl( 14 - exp, 15) + Log2_norm_lc( L_shl( t, exp ) );

    exp1 = sub( norm_l(bits_fx), 1);
    exp2 = norm_s( blen - 1 );
    b = L_shr( L_deposit_l( div_l( L_shl( bits_fx, exp1), shl( blen - 1, exp2) ) ), exp1-exp2 );

    ab = L_add( a, b);

    p_est = /*1 + */extract_l( Pow2( extract_l( L_shr(ab,15) ), ab&0x7FFF ) );

    pulses = min( p_est, MAX_PULSES );

    FOR( ; pulses >= 0; pulses--)
    {
        estbits_fx = GetBitsFromPulses_fx( pulses, blen);
        IF( L_sub( bits_fx, estbits_fx) >= 0)
        {
            BREAK;
        }
    }

    IF ( surplus_fx != 0 )
    {
        *surplus_fx = L_add(*surplus_fx, L_sub(bits_fx, estbits_fx));
    }

    return pulses;
}

void srt_vec_ind_fx (
    const Word32 *linear,      /* linear input */
    Word32 *srt,         /* sorted output*/
    Word16 *I,           /* index for sorted output  */
    Word16 length
)
{
    Word16  pos,npos,tmp;
    Word16 idxMem;
    Word32  valMem;

    /*initilize */
    FOR (pos = 0; pos < length; pos++)
    {
        I[pos] = pos;
        move16();
    }

    Copy32(linear, srt, length);

    /* now iterate */
    tmp = sub(length, 1);
    FOR (pos = 0; pos < tmp; pos++)
    {
        FOR (npos = add(pos, 1); npos < length;  npos++)
        {
            IF (L_sub(srt[npos], srt[pos]) < 0)
            {
                idxMem    = I[pos];
                move16();
                I[pos]    = I[npos];
                move16();
                I[npos]   = idxMem;
                move16();

                valMem    = L_add(srt[pos], 0);
                srt[pos]  = srt[npos];
                move32();
                srt[npos] = valMem;
                move32();
            }
        }
    }

    return;
}
