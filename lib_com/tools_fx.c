/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"           /* Compilation switches                   */
#include "prot_fx.h"           /* Function prototypes                    */
#include "basop_util.h"
#include "rom_com_fx.h" /* Function prototypes                    */
#include "cnst_fx.h"           /* Function prototypes                    */
#include "stl.h"

#define INV_BANDS10 3277 /* 1/10 in Q15 */
#define INV_BANDS9  3641 /* 1/9  in Q15 */
#define INV_BANDS3  10923 /* 1/9  in Q15 */

/*-------------------------------------------------------------------*
  * usdequant_fx()
  *
  * Uniform scalar de-quantizer routine
  *
  * Applies de-quantization based on scale and round operations.
  *-------------------------------------------------------------------*/
Word16 usdequant_fx( /* Qx*/
    const Word16 idx, /* i: quantizer index Q0*/
    const Word16 qlow, /* i: lowest codebook entry (index 0) Qx*/
    const Word16 delta /* i: quantization step Qx-1*/
)
{
    Word16 g;
    Word32 L_tmp;

    /*g = idx * delta + qlow;*/
    L_tmp = L_deposit_l(qlow);/*Qx */
    L_tmp = L_mac(L_tmp,idx,delta);/*Qx */
    g = round_fx(L_shl(L_tmp,16)); /*Qx */

    return( g );
}

/*-------------------------------------------------------------------*
 * usquant()
 *
 * Uniform scalar quantizer according to MMSE criterion
 * (nearest neighbour in Euclidean space)
 *
 * Applies quantization based on scale and round operations.
 * Index of the winning codeword and the winning codeword itself are returned.
 *-------------------------------------------------------------------*/
Word16 usquant_fx(          /* o: index of the winning codeword   */
    const Word16 x,      /* i: scalar value to quantize        Qx*/
    Word16 *xq,    /* o: quantized value                 Qx*/
    const Word16 qlow,   /* i: lowest codebook entry (index 0) Qx*/
    const Word16 delta,  /* i: quantization step               Qx-1*/
    const Word16 cbsize  /* i: codebook size                   */
)
{
    Word16 idx;
    Word16 tmp, exp;
    Word32 L_tmp;

    /*    idx = (short)( (x - qlow)/delta + 0.5f); */
    exp = norm_s(delta);
    tmp = div_s(shl(1,sub(14,exp)),delta); /*Q(29-exp-(Qx-1))->Q(30-exp-Qx) */
    L_tmp = L_mult(sub(x,qlow),tmp); /*Q(31-exp) */
    idx = extract_l(L_shr_r(L_add(L_tmp,shl(1,sub(30,exp))),sub(31,exp))); /*Q0 */

    idx = s_min(idx,sub(cbsize,1));
    idx = s_max( idx, 0);

    /*    *xq = idx*delta + qlow; */
    L_tmp = L_deposit_l(qlow);/*Qx */
    L_tmp = L_mac(L_tmp,idx,delta);/*Qx */
    *xq = round_fx(L_shl(L_tmp,16));/*Qx */

    return idx;
}
/*-------------------------------------------------------------------*
* Dot_product:
*
* Compute scalar product of <x[],y[]> using accumulator.
* Performs no normalization, as opposed to Dot_product12()
*-------------------------------------------------------------------*/
Word32 Dot_product(     /* o  : Sum              */
    const Word16 x[],   /* i  : 12bits: x vector */
    const Word16 y[],   /* i  : 12bits: y vector */
    const Word16 lg     /* i  : vector length    */
)
{
    Word16 i;
    Word32 L_sum;

    L_sum = L_mac(1L, x[0], y[0]);
    FOR (i = 1; i < lg; i++)
    {
        L_sum = L_mac(L_sum, x[i], y[i]);
    }
    return L_sum;
}
/*---------------------------------------------------------------------*
 * dotp_fx()
 *
 * Dot product of vector x[] and vector y[]
 *---------------------------------------------------------------------*/

Word32 dotp_fx(           /* o  : dot product of x[] and y[]    */
    const Word16  x[],  /* i  : vector x[]                    */
    const Word16  y[],  /* i  : vector y[]                    */
    const Word16  n,    /* i  : vector length                 */
    Word16  *exp  /* (o)    : exponent of result (0..+30)       */
)
{
    Word16 sft;
    Word32 L_sum;

    assert(*exp == 0);

    L_sum = L_add(L_shr(Dot_product(x, y, n), 1), 1);

    /* Normalize acc in Q31 */

    sft = norm_l(L_sum);
    L_sum = L_shl(L_sum, sft);

    *exp = sub(30, sft);
    move16();  /* exponent = 0..30 */

    return L_sum;
}

Word32 sum2_fx(          /* o  : sum of all squared vector elements    Q(2x+1)*/
    const Word16 *vec,  /* i  : input vector                          Qx*/
    const Word16 lvec   /* i  : length of input vector                */
)
{
    Word16 i;
    Word32 L_tmp;
    L_tmp = L_deposit_l(0);
    FOR( i=0; i<lvec; i++ )
    {
        L_tmp = L_mac(L_tmp,vec[i],vec[i]); /*Q(2x+1) */
    }

    return L_tmp;
}

/*-------------------------------------------------------------------*
 * Copy:
 *
 * Copy vector x[] to y[]
 *-------------------------------------------------------------------*/
void Copy(
    const Word16 x[],  /* i  : input vector  */
    Word16 y[],  /* o  : output vector */
    const Word16 L     /* i  : vector length */
)
{
    Word16 i;

    IF (y < x)
    {
        FOR (i = 0; i < L; i++)
        {
            y[i] = x[i];
            move16();
        }
    }
    ELSE
    {
        FOR (i = L-1; i >= 0; i--)
        {
            y[i] = x[i];
            move16();
        }
    }
}

/*-------------------------------------------------------------------*
 * Copy32:
 *
 * Copy vector x[] to y[] (32 bits)
 *-------------------------------------------------------------------*/
void Copy32(
    const Word32 x[],  /* i  : input vector  */
    Word32 y[],  /* o  : output vector */
    const Word16 L     /* i  : vector length */
)
{
    Word16 i;
    IF(y < x)
    {
        FOR (i = 0; i < L; i++)
        {
            y[i] = x[i];
            move32();
        }
    }
    ELSE
    {
        FOR (i = L-1; i >= 0; i--)
        {
            y[i] = x[i];
            move32();
        }
    }
}
/*-------------------------------------------------------------------*
  * set16_fx()
  * set32_fx()
  *
  * Set the vector elements to a value
  *-------------------------------------------------------------------*/
void set16_fx(
    Word16 y[],  /* i/o: Vector to set                       */
    const Word16 a,    /* i  : Value to set the vector to          */
    const Word16 N     /* i  : Lenght of the vector                */
)
{
    Word16 i;

    FOR (i=0 ; i<N ; i++)
    {
        y[i] = a;
        move16();
    }

    return;
}

void set32_fx(
    Word32 y[],  /* i/o: Vector to set                       */
    const Word32 a,    /* i  : Value to set the vector to          */
    const Word16 N     /* i  : Lenght of the vector                */
)
{
    Word16 i, tmp;
    tmp = extract_l(a);
    IF (L_sub(L_deposit_l(tmp), a) == 0)
    {
        FOR (i=0 ; i<N ; i++)
        {
            y[i] = L_deposit_l(tmp);
        }
    }
    ELSE
    {
        FOR (i=0 ; i<N ; i++)
        {
            y[i] = a;
            move32();
        }
    }

    return;
}
/*-------------------------------------------------------------------*
 * Copy_Scale_sig
 *
 * Up/down scale a 16 bits vector x and move it into y
 *-------------------------------------------------------------------*/
void Copy_Scale_sig(
    const Word16 x[],   /* i  : signal to scale input           Qx        */
    Word16 y[],   /* o  : scaled signal output            Qx        */
    const Word16 lg,    /* i  : size of x[]                     Q0        */
    const Word16 exp0   /* i  : exponent: x = round(x << exp)   Qx ?exp  */
)
{
    Word16 i;
    Word16 tmp;

    IF (exp0 == 0)
    {
        FOR (i = 0; i < lg; i++)
        {
            y[i] = x[i];
            move16();
        }
        return;
    }
    IF (exp0 < 0)
    {
        tmp = shl(-32768, exp0); /* we use negative to correctly represent 1.0 */
        FOR (i = 0; i < lg; i++)
        {
            y[i] = msu_r(0, x[i], tmp);
            move16();
        }
        return;
    }
    FOR (i = 0; i < lg; i++)
    {
        y[i] = shl(x[i], exp0);
        move16();/* saturation can occur here */
    }
}
/*-------------------------------------------------------------------*
 * Copy_Scale_sig
 *
 * Up/down scale a 16 bits vector x and move it into y
 *-------------------------------------------------------------------*/
void Copy_Scale_sig_16_32(
    const Word16 x[],   /* i  : signal to scale input           Qx        */
    Word32 y[],   /* o  : scaled signal output            Qx        */
    const Word16 lg,    /* i  : size of x[]                     Q0        */
    const Word16 exp0   /* i  : exponent: x = round(x << exp)   Qx ?exp  */
)
{
    Word16 i;
    Word16 tmp;


    IF (exp0 == 0)
    {
        FOR (i = 0; i < lg; i++)
        {
            y[i] = L_deposit_l(x[i]);
        }
        return;
    }
    IF (exp0 < 0)
    {
        /*Should not happen */
        FOR (i = 0; i < lg; i++)
        {
            y[i] = L_deposit_l(shl(x[i], exp0));
        }
        return;
    }
    tmp = shl(1,exp0);
    FOR (i = 0; i < lg; i++)
    {
        y[i] = L_mult0(x[i], tmp);
        move32();/* saturation can occur here */
    }
}
void Copy_Scale_sig_32_16(
    const Word32 x[],   /* i  : signal to scale input           Qx        */
    Word16 y[],   /* o  : scaled signal output            Qx        */
    const Word16 lg,    /* i  : size of x[]                     Q0        */
    const Word16 exp0   /* i  : exponent: x = round(x << exp)   Qx ?exp  */
)
{
    Word16 i;
    Word16 tmp;

    tmp = add(16,exp0);
    IF(tmp != 0)
    {
        FOR (i = 0; i < lg; i++)
        {
            y[i] = round_fx(L_shl(x[i], tmp));
        }
    }
    ELSE
    {
        FOR (i = 0; i < lg; i++)
        {
            y[i] = round_fx(x[i]);
        }
    }
}

/*-------------------------------------------------------------------*
 * Scale_sig32
 *
 * Up/down scale a 32 bits vector
 *-------------------------------------------------------------------*/
void Scale_sig32(
    Word32 x[],  /* i/o: signal to scale                 Qx        */
    const Word16 lg,   /* i  : size of x[]                     Q0        */
    const Word16 exp0  /* i  : exponent: x = round(x << exp)   Qx ?exp  */
)
{
    Word16 i;

    FOR (i = 0; i < lg; i++)
    {
        x[i] = L_shl(x[i], exp0);
        move32(); /* saturation can occur here */
    }
}

/*------------------------------------------------------------------*
 * function Random_Fill
 *
 * Signed 16 bits random generator.
 * (Avoids Store of Seed to Memory for 'n' Random Values and
 *  Combines Scaling Operation.)
 *------------------------------------------------------------------*/
void Random_Fill(
    Word16 *seed,    /* i/o: random seed         */
    Word16 n,        /* i  : number of values    */
    Word16 *y,       /* o  : output values       */
    Word16 scaling   /* i  : scaling of values   */
)
{
    Word16 i;
    Word16 local_seed;

    local_seed = *seed;
    move16();
    FOR (i=0; i<n; i++)
    {
        local_seed = extract_l(L_mac0(13849L, local_seed, 31821));
        *y++ = shr(local_seed, scaling);
        move16();
    }
    *seed = local_seed;
    move16();
}
/*-------------------------------------------------------------------*
 * Scale_sig
 * Up/down scale a 16 bits vector
 *-------------------------------------------------------------------*/
void Scale_sig(
    Word16 x[],  /* i/o: signal to scale                 Qx        */
    const Word16 lg,   /* i  : size of x[]                     Q0        */
    const Word16 exp0  /* i  : exponent: x = round(x << exp)   Qx ?exp  */
)
{
    Word16 i;
    Word16 tmp;
    IF (exp0 > 0)
    {
        FOR (i = 0; i < lg; i++)
        {
            x[i] = shl(x[i], exp0);
            move16(); /* saturation can occur here */
        }
        return;
    }
    IF (exp0 < 0)
    {
        BASOP_SATURATE_WARNING_OFF
        tmp = shl(-32768, exp0); /* we use negative to correctly represent 1.0 */
        BASOP_SATURATE_WARNING_ON
        FOR (i = 0; i < lg; i++)
        {
            x[i] = msu_r(0, x[i], tmp);
            move16(); /* msu instead of mac because factor is negative */
        }
        return;
    }
}

/*---------------------------------------------------------------------*
  * mean()
  *
  *---------------------------------------------------------------------*/
Word16 mean_fx(            /* o  : mean of vector                         */
    const Word16 *vec_fx,  /* i  : input vector                           */
    const Word16 lvec_fx   /* i  : length of input vector                 */
)
{
    Word16 tmp;
    tmp = sum16_fx(vec_fx,lvec_fx);
    tmp = mult_r(tmp,div_s(1,lvec_fx));

    return tmp;
}
/*-------------------------------------------------------------------*
 * Vr_add
 *
 * Add two Word16 vectors together integer by integer
 *-------------------------------------------------------------------*/
void Vr_add(
    const Word16 *in1, /* i  : Input vector 1                                   */
    const Word16 *in2, /* i  : Input vector 2                                   */
    Word16 *out, /* o  : Output vector that contains vector 1 + vector 2  */
    Word16 Len   /* i  : Vector lenght                                    */
)
{
    Word16 i;

    FOR (i=0; i<Len; i++)
    {
        out[i] = add(in1[i], in2[i]);
        move16();
    }
}

void sort_fx(
    Word16 *r,  /* i/o: Vector to be sorted in place */
    Word16 lo,  /* i  : Low limit of sorting range   */
    Word16 up   /* I  : High limit of sorting range  */
)
{
    Word16 i, j, i1;
    Word16 tempr;

    FOR (i=sub(up, 1); i>=lo; i--)
    {
        i1 = add(i, 1);
        tempr = r[i];
        move16();
        move16(); /*supplementary move for the j-1 PTR initialization*/
        FOR (j=i1; j<=up; j++)
        {
            IF (sub(tempr, r[j]) <= 0)
            {
                BREAK;
            }

            r[j-1] = r[j];
            move16();
        }
        r[j-1] = tempr;
        move16();
    }
}

void sort_32_fx(
    Word32 *r,    /* i/o: Vector to be sorted in place */
    const Word16 lo,    /* i  : Low limit of sorting range   */
    const Word16 up     /* I  : High limit of sorting range  */
)
{
    Word16 i, j;
    Word32 tempr;
    FOR ( i=sub(up, 1); i>=lo; i-- )
    {
        tempr = r[i];
        move32();
        FOR ( j=add(i, 1); j<=up; j++ )
        {
            IF (L_sub(tempr,r[j]) <= 0)
            {
                BREAK;
            }
            r[j-1] = r[j];
            move32();
        }

        r[j-1] = tempr;
        move32();
    }

    return;
}

Word16 minimum_fx(           /* o  : index of the minimum value in the input vector */
    const Word16 *vec_fx,  /* i  : input vector                                   */
    const Word16 lvec_fx,  /* i  : length of input vector                         */
    Word16 *min_fx   /* o  : minimum value in the input vector              */
)
{
    Word16 j, ind;
    Word16 tmp;
    ind = 0;
    move16();
    tmp = vec_fx[0];
    move16();

    FOR ( j=1 ; j<lvec_fx ; j++ )
    {
        if( sub(vec_fx[j],tmp) < 0 )
        {
            ind = j;
            move16();
            /*tmp = vec_fx[j];    move16(); */
        }
        tmp = s_min(tmp,vec_fx[j]);
    }

    *min_fx = tmp;
    move16();

    return ind;
}

Word16 maximum_fx(         /* o  : index of the maximum value in the input vector */
    const Word16 *vec_fx,  /* i  : input vector                                   */
    const Word16 lvec_fx,  /* i  : length of input vector                         */
    Word16 *max_fx   /* o  : maximum value in the input vector              */
)
{
    Word16 j, ind;
    Word16 tmp;
    ind = 0;
    move16();
    tmp = vec_fx[0];
    move16();

    FOR ( j=1 ; j<lvec_fx ; j++ )
    {
        if( sub(vec_fx[j],tmp) > 0 )
        {
            ind = j;
            move16();
        }
        tmp = s_max(tmp,vec_fx[j]);
    }
    *max_fx = tmp;
    move16();

    return ind;
}

/*---------------------------------------------------------------------*
  * minimum_32_fx()
  *
  * Find index and value of the minimum in a vector
  *---------------------------------------------------------------------*/

Word16 minimum_32_fx(      /* o  : index of the minimum value in the input vector */
    const Word32 *vec_fx,  /* i  : input vector                                   */
    const Word16 lvec_fx,  /* i  : length of input vector                         */
    Word32 *min_fx   /* o  : minimum value in the input vector              */
)
{
    Word16 j, ind;
    Word32 tmp;
    ind = 0;
    move16();
    tmp = vec_fx[0];
    move16();

    FOR ( j=1 ; j<lvec_fx ; j++ )
    {
        if( L_sub(vec_fx[j],tmp) < 0 )
        {
            ind = j;
            move16();
            /*tmp = vec_fx[j];    move32(); */
        }
        tmp = L_min(tmp,vec_fx[j]);
    }

    *min_fx = tmp;
    move32();

    return ind;
}

/*---------------------------------------------------------------------*
  * maximum_32_fx()
  *
  * Find index and value of the maximum in a vector
  *---------------------------------------------------------------------*/

Word16 maximum_32_fx(         /* o  : index of the maximum value in the input vector */
    const Word32 *vec,  /* i  : input vector                                   */
    const Word16 lvec,  /* i  : length of input vector                         */
    Word32 *max   /* o  : maximum value in the input vector              */
)
{
    Word16 j, ind;
    Word32 tmp;
    ind = 0;
    move16();
    tmp = vec[0];
    move16();

    FOR ( j=1 ; j<lvec ; j++ )
    {
        if( L_sub(vec[j],tmp) > 0 )
        {
            ind = j;
            move16();
        }
        tmp = L_max(tmp,vec[j]);
    }
    *max = tmp;
    move32();

    return ind;
}

/*----------------------------------------------------------------
 *Function:
 *Finds number of shifts to normalize a 16-bit array variable.
 *Return value
 *Number of shifts
 *----------------------------------------------------------------*/
Word16 Exp16Array(
    const Word16  n,     /* (i): Array size   */
    const Word16  *sx    /* (i): Data array   */
)
{
    Word16  k;
    Word16  exp;
    Word16  sMax;
    Word16  sAbs;

    sMax = abs_s( sx[0] );
    move16();

    FOR ( k = 1; k < n; k++ )
    {
        sAbs = abs_s( sx[k] );
        sMax = s_max( sMax, sAbs );
    }

    exp = norm_s( sMax );
    return exp;
}

Word16 Exp32Array(
    const Word16  n,     /* (i): Array size   */
    const Word32  *sx    /* (i): Data array   */
)
{
    Word16  k;
    Word16  exp;
    Word32  L_Max;
    Word32  L_Abs;

    L_Max = L_abs( sx[0] );
    FOR ( k = 1; k < n; k++ )
    {
        L_Abs = L_abs( sx[k] );
        L_Max = L_max( L_Max, L_Abs );
    }
    exp = norm_l( L_Max );

    if(L_Max == 0)
    {
        exp = 31;
        move16();
    }
    return exp;
}

Word32 sum16_32_fx(     /* o  : sum of all vector elements            Qx*/
    const Word16 *vec,  /* i  : input vector                          Qx*/
    const Word16 lvec   /* i  : length of input vector                */
)
{
    Word16 i;
    Word32 tmp, L_var;
    tmp = 0;
    move16();
    FOR( i=0; i<lvec; i++ )
    {
        L_var = L_deposit_l(vec[i]);
        tmp = L_add(tmp, L_var); /*Qx */
    }

    return tmp;
}

Word32 var_fx_32(                /* o: variance of vector                    Qx+16*/
    const Word16 *x,        /* i: input vector                          Qx*/
    const Word16 Qx,
    const Word16 len          /* i: length of inputvector                 */
)
{
    Word16 m;
    Word32 v;
    Word16 i;
    Word16 tmp, exp, inv_len;
    Word32 L_tmp;

    L_tmp = L_add(x[0], 0);
    FOR(i=1; i<len; i++)
    {
        L_tmp = L_add(L_tmp,x[i]); /*Qx */
    }
    exp = norm_s(len);
    inv_len = div_s(shl(1,sub(14,exp)),len); /*Q(29-exp) */
    L_tmp = Mult_32_16(L_tmp,inv_len); /*Q(14-exp+Qx) */
    m = round_fx(L_shl(L_tmp,add(exp,2))); /*Qx */

    v = L_deposit_l(0);
    FOR (i = 0; i < len; i++)
    {
        tmp = sub(x[i],m); /*Qx */
        v = L_mac0(v,tmp,tmp); /*(Qx+Qx) */
    }
    L_tmp = Mult_32_16(v,inv_len); /*Q(14-exp+Qx+Qx) */
    v = L_shl(L_tmp,add(exp,sub(2,Qx))); /*Qx+16 */

    return v;
}
/*-------------------------------------------------------------------*
 * conv()
 *
 * Convolution between vectors x[] and h[] written to y[]
 * All vectors are of length L. Only the first L samples of the
 * convolution are considered.
 *-------------------------------------------------------------------*/

void conv_fx(
    const Word16 x[],   /* i  : input vector                              Q_new*/
    const Word16 h[],   /* i  : impulse response (or second input vector) Q(15)*/
    Word16 y[],   /* o  : output vetor (result of convolution)      12 bits*/
    const Word16 L      /* i  : vector size                               */
)
{

    Word16 i, n;
    Word32 L_sum;

    y[0] = mult_r(x[0], h[0]);
    move16();
    FOR (n = 1; n < L; n++)
    {
        L_sum = L_mult(x[0], h[n]);
        FOR (i = 1; i < n; i++)
        {
            L_sum = L_mac(L_sum, x[i], h[n - i]);
        }
        y[n] = mac_r(L_sum, x[i], h[0]);
        move16();
    }
}

Word16 var_fx(                /* o: variance of vector                    Qx*/
    const Word16 *x,        /* i: input vector                          Qx*/
    const Word16 Qx,
    const Word16 len          /* i: length of inputvector                 */
)
{
    Word16 m;
    Word32 v;
    Word16 v_16;
    Word16 i;
    Word16 tmp, exp, inv_len;
    Word32 L_tmp;

    L_tmp = x[0];
    FOR(i=1; i<len; i++)
    {
        L_tmp = L_add(L_tmp,x[i]); /*Qx */
    }
    exp = norm_s(len);
    inv_len = div_s(shl(1,sub(14,exp)),len); /*Q(29-exp) */
    L_tmp = Mult_32_16(L_tmp,inv_len); /*Q(14-exp+Qx) */
    m = round_fx(L_shl(L_tmp,add(exp,2))); /*Qx */

    v = L_deposit_l(0);
    FOR (i = 0; i < len; i++)
    {
        tmp = sub(x[i],m); /*Qx */
        v = L_mac0(v,tmp,tmp); /*(Qx+Qx) */
    }
    L_tmp = Mult_32_16(v,inv_len); /*Q(14-exp+Qx+Qx) */
    v_16 = round_fx(L_shl(L_tmp,add(exp,sub(2,Qx)))); /*Qx */

    return v_16;
}

/*---------------------------------------------------------------------*
 * std_fx()
 *
 * Calculate the standard deviation of a vector
 *---------------------------------------------------------------------*/

Word16 std_fx(                       /* o: standard deviation                    */
    const Word16 x[],                /* i: input vector                          */
    const Word16 len                 /* i: length of the input vector            */
)
{
    Word16 i;
    Word32 L_tmp;
    Word16 exp1, exp2, tmp;
    Word32 stdev;

    stdev = 0;
    move16();
    FOR ( i = 0; i < len; i++ )
    {
        L_tmp = L_mult(x[i], x[i]);/*29 */
        stdev = L_add(stdev, L_shr(L_tmp, 3));/*26 */
    }

    IF (stdev != 0)
    {
        exp1 = norm_l(stdev);
        tmp = div_s(16384, extract_h(L_shl(stdev, exp1)));/*15 + 14 - (26 + exp1 - 16) */
        L_tmp = L_mult(tmp, len);/*15 + 14 - (26 + exp1 - 16) + 1 */
        exp2 = norm_l(L_tmp);
        exp1 = add(sub(exp1, exp2), 11);
        stdev = Isqrt_lc(L_shl(L_tmp, exp2), &exp1);/*31-exp1 */
        stdev = L_shl(stdev, sub(exp1, 1));/*30 */
    }


    return extract_h(stdev);
}

Word32 dot_product_mat_fx(    /* o  : the dot product x'*A*x        */
    const Word16  *x,      /* i  : vector x                     Q15 */
    const Word32  *A,      /* i  : matrix A                     Q0*/
    const Word16  m      /* i  : vector & matrix size          */

)
{
    Word16 i,j;
    Word32 suma,tmp_sum;
    const Word32 *pt_A;
    const Word16 *pt_x;

    pt_A = A;
    suma = L_deposit_l(0);

    FOR(i=0; i<m; i++)
    {
        tmp_sum = L_deposit_l(0);
        pt_x = x;
        FOR(j=0; j<m; j++)
        {
            tmp_sum = Madd_32_16(tmp_sum,*pt_A,*pt_x);   /*Q0 */
            pt_A++;
            pt_x++;
        }
        suma = Madd_32_16(suma,tmp_sum,x[i]);          /*Q0 */
    }
    return suma;
}
/*-------------------------------------------------------------------*
 *  Vr_subt
 *
 *  Subtract two Word16 vectors integer by integer
 *-------------------------------------------------------------------*/
void Vr_subt(
    const Word16 x1[],  /* i  : Input vector 1                                   */
    const Word16 x2[],  /* i  : Input vector 2                                   */
    Word16 y[],   /* o  : Output vector that contains vector 1 - vector 2  */
    Word16 N      /* i  : Vector lenght                                    */
)
{
    Word16 i;

    FOR (i=0; i<N ; i++)
    {
        y[i] = sub(x1[i], x2[i]);
        move16();
    }
}
/*-------------------------------------------------------------------*
 * vquant()
 *
 * Vector quantizer according to MMSE criterion (nearest neighbour in Euclidean space)
 *
 * Searches a given codebook to find the nearest neighbour in Euclidean space.
 * Index of the winning codevector and the winning vector itself are returned.
 *-------------------------------------------------------------------*/
Word16 vquant_fx(                 /* o: index of the winning codevector     */
    Word16 x[],              /* i: vector to quantize        Q13  */
    const Word16 x_mean[],         /* i: vector mean to subtract (0 if none)Q13*/
    Word16 xq[],             /* o: quantized vector                  Q13  */
    const Word16 cb[],             /* i: codebook                          Q13 */
    const Word16 dim,            /* i: dimension of codebook vectors       */
    const Word16 cbsize         /* i: codebook size                       */
)
{
    Word16  tmp;
    Word16 c, d, idx, j;
    Word32 L_dist,/*L_tmp,*/L_mindist;

    idx = 0;
    move16();
    L_mindist = MAX_32;
    move16();
    IF (x_mean != 0)
    {
        FOR( d = 0; d < dim; d++ )
        {
            /*x[d] -= x_mean[d]; */
            x[d] = sub(x[d],x_mean[d]);
            move16();/*Qx */
        }
    }
    j = 0;
    move16();
    FOR ( c = 0; c < cbsize; c++ )
    {
        L_dist = 0;
        move16();

        FOR( d = 0; d < dim; d++ )
        {
            /*tmp = x[d] - cb[j++];*/
            tmp = sub(x[d] , cb[j++]);/*Qx */
            L_dist = L_mac0(L_dist, tmp, tmp);
        }
        if ( L_sub(L_dist,L_mindist) < 0)
        {
            idx = c;
            move16();
        }
        L_mindist = L_min(L_mindist, L_dist);

    }
    if ( xq == 0 )
    {
        return idx;
    }

    /*j =  idx*dim;*/
    j =  i_mult2(idx,dim);
    FOR ( d = 0; d < dim; d++)
    {
        xq[d] = cb[j++];
        move16();
    }
    IF (x_mean != 0)
    {
        FOR( d = 0; d < dim; d++)
        {
            /*xq[d] += x_mean[d]; */
            xq[d] = add(xq[d],x_mean[d]);
            move16();
        }
    }

    return idx;
}

/*-------------------------------------------------------------------*
 * w_vquant_fx()
 *
 * Vector quantizer according to MMSE criterion (nearest neighbour in Euclidean space)
 *
 * Searches a given codebook to find the nearest neighbour in Euclidean space.
 * Weights are put on the error for each vector element.
 * Index of the winning codevector and the winning vector itself are returned.
 *-------------------------------------------------------------------*/
Word16 w_vquant_fx(
    Word16 x[],         /* i: vector to quantize in Q10 */
    Word16 Qx,
    const Word16 weights[],   /* i: error weights in Q0 */
    Word16 xq[],        /* o: quantized vector in Q15 */
    const Word16 cb[],        /* i: codebook in Q15 */
    const Word16 cbsize,      /* i: codebook size */
    const Word16 rev_vect     /* i: reverse codebook vectors */
)
{
    Word16 tmp;
    Word16 c, idx, j;
    Word32 dist, minDist;

    idx = 0;
    move16();
    minDist = 0x7fffffffL;
    move32();
    Qx = sub(15, Qx);

    j = 0;
    move16();
    IF (rev_vect)
    {
        FOR ( c = 0; c < cbsize; c++)
        {
            dist = L_deposit_l(0);

            tmp = sub(x[3], shr(cb[j++], Qx));
            if (weights[3] != 0)
            {
                dist = L_mac0(dist, tmp, tmp);
            }
            tmp = sub(x[2], shr(cb[j++], Qx));
            if (weights[2] != 0)
            {
                dist = L_mac0(dist, tmp, tmp);
            }
            tmp = sub(x[1], shr(cb[j++], Qx));
            if (weights[1] != 0)
            {
                dist = L_mac0(dist, tmp, tmp);
            }
            tmp = sub(x[0], shr(cb[j++], Qx));
            if (weights[0] != 0)
            {
                dist = L_mac0(dist, tmp, tmp);
            }

            if (L_sub(dist, minDist) < 0)
            {
                idx = c;
                move16();
            }
            minDist = L_min(minDist, dist);
        }

        IF (xq == 0)
        {
            return idx;
        }

        j =  shl(idx, 2);
        xq[3] = cb[j++];
        move16();/* in Q15 */
        xq[2] = cb[j++];
        move16();/* in Q15 */
        xq[1] = cb[j++];
        move16();/* in Q15 */
        xq[0] = cb[j++];
        move16();/* in Q15 */
    }
    ELSE
    {
        FOR ( c = 0; c < cbsize; c++)
        {
            dist = L_deposit_l(0);

            tmp = sub(x[0], shr(cb[j++], Qx));
            if (weights[0] != 0)
            {
                dist = L_mac0(dist, tmp, tmp);
            }
            tmp = sub(x[1], shr(cb[j++], Qx));
            if (weights[1] != 0)
            {
                dist = L_mac0(dist, tmp, tmp);
            }
            tmp = sub(x[2], shr(cb[j++], Qx));
            if (weights[2] != 0)
            {
                dist = L_mac0(dist, tmp, tmp);
            }
            tmp = sub(x[3], shr(cb[j++], Qx));
            if (weights[3] != 0)
            {
                dist = L_mac0(dist, tmp, tmp);
            }

            if (L_sub(dist, minDist) < 0)
            {
                idx = c;
                move16();
            }
            minDist = L_min(minDist, dist);
        }

        IF (xq == 0)
        {
            return idx;
        }

        j =  shl(idx, 2);
        xq[0] = cb[j++];
        move16(); /* in Q15 */
        xq[1] = cb[j++];
        move16(); /* in Q15 */
        xq[2] = cb[j++];
        move16(); /* in Q15 */
        xq[3] = cb[j++];
        move16(); /* in Q15 */
    }

    return idx;
}
/*-------------------------------------------------------------------*
 * Emaximum:
 *
 * Find index of a maximum energy in a vector
 *-------------------------------------------------------------------*/
Word16 emaximum_fx(               /* o  : return index with max energy value in vector  Q0 */
    const Word16 Qvec,          /* i  : Q of input vector                         Q0 */
    const Word16 *vec,          /* i  : input vector                              Qx */
    const Word16 lvec,          /* i  : length of input vector                    Q0 */
    Word32 *ener_max      /* o  : maximum energy value                      Q0 */
)
{
    Word16 j, ind;
    Word32 L_tmp, L_tmp1;
    Word32 emax;

    emax = L_mult0(vec[0], vec[0]);
    ind = 0;
    move16();

    FOR (j = 1; j<lvec; j++)
    {
        L_tmp = L_mult0(vec[j], vec[j]);
        L_tmp1 = L_sub(L_tmp, emax);
        if (L_tmp1 > 0)
        {
            ind = j;
            move16();
        }
        emax = L_max(emax, L_tmp);
    }

    *ener_max = L_shr(emax, add(Qvec, Qvec));
    move32();

    return ind;
}
/*-------------------------------------------------------------------*
 * mean32:
 *
 * Find the mean of a 32 bits vector
 *-------------------------------------------------------------------*/
Word32 Mean32(              /* o  : mean of the elements of the vector */
    const Word32 in[],      /* i  : input vector                       */
    const Word16 L          /* i  : length of input vector             */
)
{
    Word32 Ltmp;
    Word16 inv_L;

    inv_L = INV_BANDS9;
    move16();
    if (sub(L, 10) == 0)
    {
        inv_L = INV_BANDS10;
        move16();
    }

    Ltmp = sum32_fx(in, L);

    Ltmp = Mult_32_16(Ltmp, inv_L);

    return Ltmp;
}
Word32 sum32_fx(           /* o  : sum of all vector elements            Qx*/
    const Word32 *vec,  /* i  : input vector                          Qx*/
    const Word16 lvec   /* i  : length of input vector                */
)
{
    Word16 i;
    Word32 tmp;
    tmp = L_deposit_l(0);
    FOR( i=0; i<lvec; i++ )
    {
        tmp = L_add(tmp,vec[i]); /*Qx */
    }

    return tmp;
}
Word16 sum16_fx(           /* o  : sum of all vector elements            Qx*/
    const Word16 *vec,  /* i  : input vector                          Qx*/
    const Word16 lvec   /* i  : length of input vector                */
)
{
    Word16 i;
    Word16 tmp;
    tmp = 0;
    move16();
    FOR( i=0; i<lvec; i++ )
    {
        tmp = add(tmp,vec[i]); /*Qx */
    }

    return tmp;
}
/*------------------------------------------------------------------*
 * function Random
 *
 * Signed 16 bits random generator.
 *------------------------------------------------------------------*/
Word16 Random(       /* o  : output random value */
    Word16 *seed     /* i/o: random seed         */
)
{
    *seed = extract_l(L_mac0(13849L, *seed, 31821));

    return *seed;
}

Word16 own_random2_fx(Word16 seed)
{
    return extract_l(L_mac0(13849, seed, 31821));
}



/*------------------------------------------------------------------*
  * function Div_32_optmz
  *
  * Performs 32 bits interger division
  *------------------------------------------------------------------*/
static Word32 Div_32_optmz (Word32 L_num, Word16 denom_hi)
{
    Word16 approx, hi, lo, n_hi, n_lo;
    Word32 L_32;

    /* First approximation: 1 / L_denom = 1/denom_hi */

    approx = div_s ((Word16) 0x3fff, denom_hi);

    /* 1/L_denom = approx * (2.0 - L_denom * approx) */

    L_32 = L_msu ((Word32) 0x7fffffffL, denom_hi, approx);

    lo = L_Extract_lc (L_32, &hi);
    L_32 = Mpy_32_16 (hi, lo, approx);

    /* L_num * (1/L_denom) */

    lo = L_Extract_lc (L_32, &hi);
    n_lo = L_Extract_lc (L_num, &n_hi);
    L_32 = Mpy_32 (n_hi, n_lo, hi, lo);

    return (L_32);
}
/*------------------------------------------------------------------*
  * function iDiv_and_mod_32
  *
  * return the quotient and the modulo 32 bits numerator divided by a 16 bit denominator
  * The denominator might be right shifted by 1
  *------------------------------------------------------------------*/
void iDiv_and_mod_32(
    const Word32 Numer,       /* i  : 32 bits numerator   */
    const Word16 Denom,       /* i  : 16 bits denominator */
    Word32 * Int_quotient,    /* o  : integer result of the division (int)(num/den) */
    Word32 * Int_mod,         /* o  : modulo result of the division  num-((int)(num/den)*den)*/
    const Word16 rshift       /* i  : 0 if no right shift / 1 if the denom is right shifted by 1 */
)
{
    Word32 Quotient;
    Word16 expA, expB;
    Word32 N, TEMP;
    Word16 D;

    /* Normalize 'Numer' & 'Denom' */
    /* Use Temporary to Preserve the Original Value */
    expA = norm_l(Numer);
    N = L_shl(Numer, expA);
    expB = norm_s(Denom);
    D = shl(Denom, expB);

    /* Need to shift right 'Numer' by 1? */
    if (L_mac(N, D, -32768L) >= 0)
    {
        expA = sub(expA, 1);
    }
    N = L_shl(Numer, expA);

    /* Perform Approximation of the Division
     * Since 'lo' part is '0' AND 'denom' is supposed to be constant in the targeted usage
     * one could import the Div32 code and pre-calc the div_s and eliminate all calcs
     * with 'lo' to save some complexity */

    Quotient = Div_32_optmz(N, D); /* takes 36 clocks */
    /* Bring Back to Q0 (minus 2 because we removed the left shift by 2 in the Div32_optmz) */
    IF(rshift)
    {
        Quotient = L_shr(Quotient, add(15-2, sub(expA, sub(expB,1))));
    }
    ELSE
    {
        Quotient = L_shr(Quotient, add(15-2, sub(expA, expB)));
    }

    /* Cross Check (do Quotient x Divisor)
     * The Quotient is unsigned but we cannot just use extract_h because
     * extract_l on the low part will get the sign of the bit #15.
     * In a 32 bits value, what is in bits 0-15 is un unsigned 16 bits value.
     * So, we shift left by 1, extract the hi part and mult it by 32768 (hence the L_shl by 16-1.
     * Then we take 15 bits left (mask the others) and multiply by Denom.
     * Technically this could overflow. But since we are mutiplying to get
     * back to the Numer value that fitted in a 32 bits, doing Divisor x Quotient
     * must necessarily fit in a 32 bits
     * It is assumed that all are positive values. If not, one could
     * check the sign of the numer and denom, turn them into abs values
     * and restore the sign after*/
    TEMP = L_shl(L_mult0(extract_h(L_shl(Quotient, 1)), Denom), 16-1);
    TEMP = L_mac0(TEMP, extract_l(L_and(0x7FFF, Quotient)), Denom);



    /* Here we test to see if the previous "Quotient x Divisor" (or TEMP) is too small
     * that is the "Numer" minus TEMP is bigger or Equal to the Divisor.
     * If it is then the quotient is too small. We need to increase it by 1.
     * That is caused by our Div32 fractionnal division that can be off by 1
     * sometimes.
     * In some cases, when the divisor is very small (like 10 or something)
     * the quotient could be off by more than 1 and we would need a loop
     * to check again. That is not the case here with the current divisor */
    IF(rshift)
    {
        TEMP = L_shl(TEMP, 1);
        WHILE (L_msu0(L_sub(Numer, TEMP), 2, Denom) >= 0)
        {
            Quotient = L_add(Quotient, 1);
            TEMP = L_shl(L_mult0(extract_h(L_shl(Quotient, 1)), Denom), 16-1);
            TEMP = L_mac0(TEMP, extract_l(L_and(0x7FFF, Quotient)), Denom);
            TEMP = L_shl(TEMP, 1);
        }
    }
    ELSE
    {
        WHILE (L_msu0(L_sub(Numer, TEMP), 1, Denom) >= 0)
        {
            Quotient = L_add(Quotient, 1);
            TEMP = L_shl(L_mult0(extract_h(L_shl(Quotient, 1)), Denom), 16-1);
            TEMP = L_mac0(TEMP, extract_l(L_and(0x7FFF, Quotient)), Denom);
        }
    }
    *Int_quotient = Quotient;
    move32();
    IF(L_msu0(L_sub(Numer, TEMP), 1, Denom) == 0)
    {
        *Int_mod = 0L;
        move32();
    }
    ELSE
    {
        *Int_mod = L_sub(Numer, TEMP);
        move32();
    }


}

/*===================================================================*/
/* FUNCTION      :  pz_filter_sp_fx ()                                  */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Generic pole-zero filter routine, with single    */
/*                  precision memory                                 */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*                                                                   */
/*   _ (Word16 []) b   : zero filter coefficients (Qc).              */
/*   _ (Word16 []) a   : pole filter coefficients (Qc), a(0)=1 in Qc */
/*   _ (Word16 []) x   : input signal (Qn).                          */
/*   _ (Word16)    PNR   : NR filter order                           */
/*   _ (Word16)    PDR   : DR filter order                           */
/*   _ (Word16)    N   : number of input samples.                    */
/*   _ (Word16)    Qa  : Q factor compensation (Qa=16-Qc)            */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                                                                   */
/*   _ (Word16 []) y : output signal  (Qn)                           */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                                                                   */
/*   _ (Word16 []) buf : filter memory (Qn-Qa).                      */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*===================================================================*/

void pz_filter_sp_fx (
    const Word16 b [],
    const Word16 a [],
    Word16 x [],
    Word16 y [],
    Word16 buf [],
    Word16 PNR,
    Word16 PDR,
    Word16 N,
    Word16 Qa
)
{
    Word16 i, j;
    Word16 s;
    Word16 s_mem;
    Word32 Ltemp1;
    Word32 Lacc;

    s = negate( Qa );
    s = add( s, s );  /* s=-2Qa*/
    s = add( s, 1 );
    FOR ( i = 0; i < N; i++ )
    {
        Lacc = L_deposit_h( x[i] ); /* Lacc in Q(16+Qn)*/
        Lacc = L_shl( Lacc, s );   /* Lacc=x[i] in Q(16+Qn-2Qa+1)*/
        FOR ( j = PDR - 1; j >= 0; j-- )
        Lacc = L_msu( Lacc, buf[j], a[j + 1] ); /*Q(16+Qn-2Qa+1)*/



        Lacc = L_shr( Lacc, 1 );
        Ltemp1 = L_add( L_shl( Lacc, Qa ), 0x08000 );
        s_mem = extract_h( Ltemp1 );

        Lacc = L_deposit_l(0);
        FOR ( j = PNR - 1; j >= 0; j-- )
        Lacc = L_mac( Lacc, buf[j], b[j + 1] );
        Lacc = L_mac( Lacc, s_mem, b[0] );
        /* Lacc in Q(1+Qc+Qn-Qa)*/

        FOR ( j = s_max(PDR, PNR ) - 1; j > 0; j-- )
        {
            /* Update filter memory */
            buf[j] = buf[j - 1];
            move16();
        }
        buf[0] = s_mem;
        move16();

        Ltemp1 = L_add( L_shr( Lacc, s ), 0x08000 ); /*  Ltemp1 in Qc+Qa+Qn=Q(Qn) */
        y[i] = extract_h( Ltemp1 );  /*  y[i] in Qn */
    }
}



Word32 root_a_fx(
    Word32 a,
    Word16 Q_a,
    Word16* exp_out
)
{
    Word16 exp, tmp;
    Word32 L_tmp;

    if ( a <= 0 )
    {
        *exp_out = 0;
        return 0;
    }

    exp = norm_l( a );
    tmp = extract_h( L_shl( a, exp ) );
    exp = sub( exp, sub(30, Q_a) );
    tmp = div_s( 16384, tmp );
    L_tmp = L_deposit_h( tmp );
    L_tmp = Isqrt_lc( L_tmp, &exp );

    *exp_out = exp;
    move16();

    return L_tmp;
}


Word32 root_a_over_b_fx(
    Word32 a,
    Word16 Q_a,
    Word32 b,
    Word16 Q_b,
    Word16* exp_out
)
{
    Word16 tmp, num, den, scale;
    Word16 exp, exp_num, exp_den;
    Word32 L_tmp;
    test();
    if ( ( a <= 0 ) || ( b <= 0 ) )
    {
        *exp_out = 0;
        move16();
        return 0;
    }

    exp_num = norm_l( b );
    num = round_fx( L_shl( b, exp_num ) );
    exp_num = sub( sub( 30, exp_num ), Q_b );

    exp_den = norm_l( a );
    den = round_fx( L_shl( a, exp_den ) );
    exp_den = sub( sub( 30, exp_den ), Q_a );

    scale = shr( sub( den, num ), 15 );
    num = shl( num, scale );
    exp_num = sub( exp_num, scale );

    tmp = div_s( num, den );
    exp = sub( exp_num, exp_den );

    L_tmp = L_deposit_h( tmp );
    L_tmp = Isqrt_lc( L_tmp, &exp );

    *exp_out = exp;
    move16();

    return L_tmp;
}

/*===================================================================*/
/* FUNCTION      :  fir_fx ()                     */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Generic FIR filter routine                       */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*                                                                   */
/*   _ (Word16 []) b   : filter coefficients (Qc).                   */
/*   _ (Word16 []) x   : input signal  (Qn).                         */
/*   _ (Word16)    P   : filter order.                               */
/*   _ (Word16)    N   : number of input samples.                    */
/*   _ (Word16)    Qa  : Q factor compensation (Qa=16-Qc)            */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                                                                   */
/*   _ (Word16 []) y : output signal  (Qn)                           */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                                                                   */
/*   _               : None                                          */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*===================================================================*/
void fir_fx( const Word16 x[],      /* i  : input vector                              Qx*/
             const Word16 h[],               /* i  : impulse response of the FIR filter        Q12*/
             Word16 y[],                     /* o  : output vector (result of filtering)       Qx*/
             Word16 mem[],                   /* i/o: memory of the input signal (L samples)    Qx*/
             const Word16 L,                 /* i  : input vector size                         */
             const Word16 K,                 /* i  : order of the FIR filter (K+1 coefs.)      */
             const Word16 upd                /* i  : 1 = update the memory, 0 = not            */
             , Word16 shift                   /* i  : difference between Q15 and scaling of h[] */
           )
{

    Word16 buf_in[L_FRAME32k+L_FILT_MAX];
    Word16 i, j;
    Word32 s;

    /* prepare the input buffer (copy and update memory) */
    Copy( mem, buf_in, K );
    Copy( x, buf_in + K, L );
    IF ( upd )
    {
        Copy( buf_in + L, mem, K );
    }

    /* do the filtering */
    FOR ( i = 0; i < L; i++ )
    {
        s = L_mult( buf_in[K + i], h[0] );

        FOR ( j = 1; j <= K; j++ )
        {
            s = L_mac( s, h[j], buf_in[K + i - j] );
        }
        s = L_shl( s, shift );
        y[i] = round_fx( s ); /*Qx */
    }
}


/*--------------------------------------------------------------------------------*/
/* squant_fx()                                                                    */
/*--------------------------------------------------------------------------------*/
Word16 squant_fx(                   /* o: index of the winning codeword   */
    const    Word16 x,               /* i: scalar value to quantize        */
    Word16* xq,             /* o: quantized value                 */
    const   Word16 cb[],            /* i: codebook                        */
    const   Word16 cbsize           /* i: codebook size                   */
)
{
    Word16 tmp;
    Word16 c, idx;
    Word32 L_mindist, L_dist;

    idx = 0;
    move16();
    L_mindist = MAX_32;
    move32();

    FOR ( c = 0; c < cbsize; c++ )
    {
        L_dist = L_deposit_l(0);
        tmp = sub( x, cb[c] );

        /*dist += tmp*tmp; */
        L_dist = L_mac( L_dist, tmp, tmp );

        if ( L_sub( L_dist, L_mindist ) < 0 )
        {
            idx = c;
            move16();
        }
        L_mindist = L_min(L_mindist, L_dist);
    }

    *xq = cb[idx];
    move16();

    return idx;
}


/*===================================================================*/
/* FUNCTION      :  pz_filter_dp_fx ()                               */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  Generic pole-zero filter routine, with double    */
/*                  precision memory, transposed direct form II      */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*                                                                   */
/*   _ (Word16 []) b   : zero filter coefficients (Qc).              */
/*   _ (Word16 []) a   : pole filter coefficients (Qc), a(0)=1       */
/*   _ (Word16 []) x   : input signal (Qx).                          */
/*   _ (Word16)    P   : filter order.                               */
/*   _ (Word16)    N   : number of input samples.                    */
/*   _ (Word16)    Qa  : Q factor compensation (Qa=16-Qc)            */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                                                                   */
/*   _ (Word16 []) y : output signal  (Qx)                           */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                                                                   */
/*   _ (Word32 []) buf : filter memory (Qx+Qc)                          */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS : _ None.                                        */
/*===================================================================*/

void pz_filter_dp_fx (
    const Word16 b [],
    const Word16 a [],
    Word16 x [],
    Word16 y [],
    Word32 buf [],
    Word16 PNR,
    Word16 PDR,
    Word16 N,
    Word16 Qa
)
{
    Word16 i, j;
    Word16 s;
    Word32 s_mem;
    Word32 Ltemp1;
    Word32 Lacc;

    s = negate( Qa );
    s = add( s, s );  /*  s=-2Qa */
    s = add( s, 1 );
    FOR ( i = 0; i < N; i++ )
    {
        Lacc = L_deposit_h( x[i] );     /* Lacc in Q(16+Qn)*/
        Lacc = L_shl( Lacc, s );        /* Lacc=x[i] in Q(16+Qn-2Qa+1)*/
        FOR ( j = PDR - 1; j >= 0; j-- )
        Lacc = Msub_32_16( Lacc, buf[j], a[j + 1] );  /*Q(16+Qn-2Qa+1)*/

        s_mem = L_shl( Lacc, sub( Qa, 1 ) );  /*Qn-Qa+16=Qn+Qc*/

        Lacc = L_deposit_l(0);
        FOR ( j = PNR - 1; j >= 0; j-- )
        Lacc = Madd_32_16( Lacc, buf[j], b[j + 1] );
        Lacc = Madd_32_16( Lacc, s_mem, b[0] );
        /* Lacc in Q(1+Qc+Qn-Qa) */

        FOR ( j = s_max( PDR, PNR ) - 1; j > 0; j-- )
        {
            /* Update filter memory */
            buf[j] = buf[j - 1];
            move16();
        }
        buf[0] = s_mem;
        move16();

        Ltemp1 = L_shr( Lacc, s ); /* Ltemp1 in Qc+Qa+Qn=Q(16+Qn) */
        y[i] = extract_h( Ltemp1 );  /* y[i] in Qn */
    }
}

/*-------------------------------------------------------------------*
 * Copy_Scale_sig32_16
 *
 * Up/down scale a 32 bits vector and round to 16 bits vector
 *-------------------------------------------------------------------*/
void Copy_Scale_sig32_16(
    const Word32 *src, /* i  : signal to scale                 Qx        */
    Word16 *dst, /* o  : scaled signal                   Qx        */
    Word16 len,  /* i  : size of x[]                     Q0        */
    Word16 exp0) /* i  : exponent: x = round(x << exp)   Qx ?exp  */
{
    Word16 i;
    Word32 L_temp;

    IF (exp0 == 0)
    {
        FOR (i = 0; i < len; i++ )
        {
            *dst++ = round_fx(*src++);
        }
        return;
    }

    FOR (i = 0; i < len; i++ )
    {
        L_temp = L_shl(*src++, exp0);

        *dst++ = round_fx(L_temp);
    }
}

/*-------------------------------------------------------------------*
 * add_vec()
 *
 * Addition of two vectors sample by sample
 *-------------------------------------------------------------------*/

void add_vec_fx(
    const Word16 x1[],   /* i  : Input vector 1                       */
    const Word16 Qx1,    /* i  : SCaling of input 1                    */
    const Word16 x2[],   /* i  : Input vector 2                       */
    const Word16 Qx2,    /* i  : SCaling of input 1                    */
    Word16 y[],    /* o  : Output vector that contains vector 1 + vector 2  */
    const Word16 Qy,     /* i  : SCaling of output 1                    */
    const Word16 N       /* i  : Vector lenght                                    */
)
{
    Word16 i, Qyx1, Qyx2;
    Qyx1 = sub(Qx1,Qy);
    Qyx2 = sub(Qx2,Qy);
    IF (Qyx1 == 0)
    {
        FOR (i=0 ; i<N ; i++)
        {
            y[i] = add(x1[i],shr_r(x2[i],Qyx2));
            move16();
        }
    }
    ELSE
    {
        FOR (i=0 ; i<N ; i++)
        {
            y[i] = add(shr_r(x1[i],Qyx1),shr_r(x2[i],Qyx2));
            move16();
        }
    }
    return;
}

/*-------------------------------------------------------------------*
 *  Add_flt32_flt32
 *
 *  Add two Pseudo Float Value that are 32 Bits Mantisa and 16 Bits Exp
 *-------------------------------------------------------------------*/
Word32 Add_flt32_flt32(/* o: Result (Normalized)                */
    Word32 a,          /* i: 1st Value                          */
    Word16 exp_a,      /* i: Exponent of 1st Value (Q of Value) */
    Word32 b,          /* i: 2nd Value                          */
    Word16 exp_b,      /* i: Exponent of 2nd Value (Q of Value) */
    Word16 *exp_out    /* o: Exponent of Result                 */
)
{
    Word16 temp, temp2;
    Word32 L_temp;

    /* Subract 1 to further divide by 2 to avoid overflow on L_add */
    temp = sub(s_min(exp_a, exp_b), 1);

    /* Put both to same exponent */
    exp_a = sub(exp_a, temp);
    a = L_shr(a, exp_a);
    exp_b = sub(exp_b, temp);
    b = L_shr(b, exp_b);

    /* add them together */
    L_temp = L_add(a, b);
    temp2 = norm_l(L_temp);

    *exp_out = add(temp, temp2);
    move16();

    return L_shl(L_temp, temp2);
}

/*-------------------------------------------------------------------*
 *  Mul_flt32_Q15
 *
 *  Multiply one Pseudo Float Value (32 Bits Mantisa and 16 Bits Exp)
 *  with a Q15 value
 *-------------------------------------------------------------------*/
Word32 Mul_flt32_Q15(  /*  o: Result (Normalized)            */
    Word32 value,      /*  i: Pseudo_float Value             */
    Word16 *exp_v,     /*i/o: Exponent of Value (Q of Value) */
    Word16 frac        /*  i: Q15 value                      */
)
{
    Word16 temp;
    Word32 L_temp;

    L_temp = Mult_32_16(value, frac);
    temp = norm_l(L_temp);

    *exp_v = add(temp, *exp_v);
    move16();

    return L_shl(L_temp, temp);
}

/*-------------------------------------------------------------------*
 *  Div_flt32_flt32
 *
 *  Divide one Pseudo Float Value (32 Bits Mantisa and 16 Bits Exp)
 *  by another one
 *-------------------------------------------------------------------*/
Word32 Div_flt32_flt32(/* o: Result (Normalized)                */
    Word32 a,          /* i: 1st Value                          */
    Word16 exp_a,      /* i: Exponent of 1st Value (Q of Value) */
    Word32 b,          /* i: 2nd Value                          */
    Word16 exp_b,      /* i: Exponent of 2nd Value (Q of Value) */
    Word16 *exp_out    /* o: Exponent of Result                 */
)
{
    Word16 temp, temp2;
    Word32 L_temp;

    temp = div_s(16384, round_fx(b));
    L_temp = Mult_32_16(a, temp);
    temp2 = sub(31-1, exp_b);
    temp2 = add(temp2, exp_a);

    temp = norm_l(L_temp);

    *exp_out = add(temp, temp2);
    move16();

    return L_shl(L_temp, temp);
}


/*-------------------------------------------------------------------*
 *  Calc_Energy_Autoscaled
 *
 *  Calculate Energy with overflow protection
 *-------------------------------------------------------------------*/
Word32 Calc_Energy_Autoscaled(/* o: Result (Energy)                  */
    const Word16 *signal,           /* i: Signal                           */
    Word16 signal_exp,        /* i: Exponent of Signal (Q of Signal) */
    Word16 len,               /* i: Frame Length                     */
    Word16 *energy_exp        /* o: Exponent of Energy (Q of Energy) */
)
{
    Word16 temp, temp2;
    Word32 L_temp, L_Energy;
    Word16 i,j;

    Overflow = 0;
    move16();

    temp2 = 0;
    move16();
    L_Energy = L_deposit_l(1);
    j = s_and(7, len);
    FOR (i = 0; i < j; i++)
    {
        /* divide by 2 so energy will be divided by 4 */
        temp = mult_r(*signal++, 16384);
        L_Energy = L_mac0(L_Energy, temp, temp);
    }
    FOR (i = j; i < len; i+=8) /* Process 8 Samples at a time */
    {
        /* divide by 2 so energy will be divided by 4 */
        temp = mult_r(*signal++, 16384);
        L_temp = L_mult0(temp, temp);
        FOR (j = 1; j < 8; j++)
        {
            temp = mult_r(*signal++, 16384);
            L_temp = L_mac0(L_temp, temp, temp);
        }

        L_temp = L_shr(L_temp, temp2);
        /* Here we try the addition just to check if we can sum
           the energy of the small (8 Iterations) loop with the
           total energy calculated so far without an overflow.
           The result is discarded. If there is an overflow both
           energies are div by 2. Otherwise, nothing is done.
           After the 'IF', the sum is done again and will always
           be without an overflow. */
        L_add(L_Energy, L_temp);
        IF (Overflow != 0)
        {
            L_Energy = L_shr(L_Energy, 1);
            L_temp = L_shr(L_temp, 1);
            temp2 = add(temp2, 1);
            Overflow = 0;
            move16();
        }
        L_Energy = L_add(L_Energy, L_temp);
    }
    /* Calc Final Exponent (sub 2 because of the mult_r by 16384 that already divs the ener by 4) */
    temp2 = sub(sub(shl(signal_exp, 1), temp2), 2);

    *energy_exp = temp2;
    move16();

    return L_Energy;
}

Word16 Find_Max_Norm16(const Word16 *src, Word16 len)
{
    Word16 i;
    Word16 max16;

    /* it starts at '0' and not '1' like in Find_Max_Norm32() */
    /* and that is necessary. */
    max16 = 0;
    move16();

    FOR (i = 0; i < len; i++ )
    {
        max16 = s_max(max16, abs_s(*src++));
    }

    return norm_s(max16);
}

Word16 Find_Max_Norm32(const Word32 *src, Word16 len)
{
    Word16 i;
    Word32 max32;

    max32 = L_deposit_l(1);

    FOR (i = 0; i < len; i++ )
    {
        max32 = L_max(max32, L_abs(*src++));
    }

    return norm_l(max32);
}

/*-------------------------------------------------------------------*
 *  Sqrt_Ratio32
 *
 *  Calculate Sqrt of Val1/Val2
 *-------------------------------------------------------------------*/
Word32 Sqrt_Ratio32(/* o: Result in Q31                                            */
    Word32 L_val1,  /* i: Mantisa of Val1                                          */
    Word16 exp1,    /* i: Exp of Val1 (>0: Val was Left Shifted, <0:Right Shifted) */
    Word32 L_val2,  /* i: Mantisa of Val2                                          */
    Word16 exp2,    /* i: Exp of Val2 (same as exp1)                               */
    Word16 *exp     /* o: Exp of Result (# of 'L_shl' Req to get to Final Value)   */
)
{
    Word16 temp;

    /* Normalize Energy #1 */
    temp = norm_l(L_val1);
    L_val1 = L_shl(L_val1, temp);
    /* Adjust Exponent of Energy #1 */
    exp1 = add(exp1, temp);

    /* Normalize Energy #2 */
    temp = norm_l(L_val2);
    L_val2 = L_shl(L_val2, temp);
    /* Adjust Exponent of Energy #1 */
    exp2 = add(exp2, temp);

    /* Prepare for Inverse */
    temp = round_fx(L_val1);
    temp = div_s(16384, temp);
    /* Mult Now */
    L_val2 = Mult_32_16(L_val2, temp);
    exp1 = add(sub(exp2, exp1), 15*2);

    /* Here Result of ('L_val2' / 2^'exp2') / ('L_val1' / 2^'exp1') is */
    /* 'L_val2' / 2^'exp1' */
    /* Which is val2/val1 instead of val1/val2 because we will use Inverted Square Root */
    /* Normalize before Square Root */
    temp = norm_l(L_val2);
    L_val2 = L_shl(L_val2, temp);
    exp1 = add(temp, exp1);
    /* Do Sqrt */
    temp = sub(31, exp1);
    L_val1 = Isqrt_lc(L_val2, &temp);

    *exp = temp;
    move16();

    return L_val1;
}

Word16 Invert16( /* result in Q'15 + 'exp' */
    Word16 val,
    Word16 *exp)
{
    Word16 temp;

    /* Normalize Value */
    temp = norm_s(val);
    val = shl(val, temp);

    *exp = sub(sub(15-1, *exp), temp);
    move16();/* -1 because of 0x4000 is 1.0 in Q14 (and not Q15) */

    temp = div_s(0x4000, val);

    return temp;
}

Word16 find_rem(Word16 n, Word16 m, Word16 *r)
{
    Word16 i, q1, q2, qd;
    Word32 Ltemp2;
    Word32 Lacc;

    test();
    test();
    IF (n<=0 || m<=0 || n<m)
    {
        *r=n;
        move16();
        return (0);
    }

    q1=norm_s(n);
    q1 = sub(q1,1);
    Lacc=L_deposit_h(shl(n,q1));
    qd=sub(q1,1);
    q2=norm_s(m);
    q2 = sub(q2,1);
    Ltemp2=L_deposit_h(shl(m,q2));
    qd=sub(q2,qd);
    q2=add(q2,1);

    FOR (i=0; i<qd; i++)
    {
        Lacc=L_sub(Lacc,Ltemp2);
        IF (Lacc>=0)
        {
            Lacc=L_add(L_shl(Lacc,1),1);
        }
        ELSE
        {
            Lacc=L_add(Lacc,Ltemp2);
            Lacc=L_shl(Lacc,1);
        }
    }
    q1=extract_l(Lacc);
    Ltemp2=L_shr(Lacc,q2);
    *r=extract_h(Ltemp2);
    return(q1);
}


Word32 find_remd(Word32 n, Word32 m, Word32 *r)
{
    Word16 i, q1, q2, qd;
    Word32 Ltemp2, qo;
    Word32 Lacc;

    test();
    test();
    IF (n<=0 || m<=0 || n<m)
    {
        *r=n;
        move16();
        return (0);
    }

    q1=norm_l(n);
    q1 = sub(q1,1);
    Lacc=L_shl(n,q1);
    qd=sub(q1,1);
    q2=norm_l(m);
    q2 = sub(q2,1);
    Ltemp2=L_shl(m,q2);
    qd=sub(q2,qd);
    q2=add(q2,1);
    qo=0;
    move16();

    FOR (i=0; i<qd; i++)
    {
        Lacc=L_sub(Lacc,Ltemp2);
        qo=L_shl(qo,1);
        IF (Lacc>=0)
        {
            Lacc=L_shl(Lacc,1);
            qo=L_add(qo,1);
        }
        ELSE
        {
            Lacc=L_add(Lacc,Ltemp2);
            Lacc=L_shl(Lacc,1);
        }
    }
    *r=L_shr(Lacc,q2);
    return(qo);
}

Word16 rint_new_fx (
    Word32 x   /*Q16 */
)
{
    Word16 a;
    Word32 L_tmp;
    Word16 frac, tmp;

    /* middle value point test */
    frac = lshr(extract_l(x),1); /*Q15 */
    tmp = sub(frac,0x4000);

    IF (!tmp)
    {
        a = add(extract_h(x),1);

        if (s_and(a,1) == 0)
        {
            return a;
        }
        if (s_and(a,1) != 0)
        {
            return extract_h(x);
        }
        return extract_h(x);
    }
    ELSE
    {
        L_tmp = L_add(x,32768); /*Q16 */
        return extract_h(L_tmp);
    }
}


/*===================================================================*/
/* FUNCTION      :  erb_diff_search_fx ()                            */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  erb amplitude VQ search for QPPP                 */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*   _ (Word16 []) prev_erb : Previous erb amplitude, Q13            */
/*   _ (Word16 []) curr_erb : Current erb amplitude, Q13             */
/*   _ (Word16 []) dif_erb: erb differential, Q13                    */
/*   _ (Word16 []) pow_spec : LPC power spectrum, Q7                 */
/*   _ (Word16 [][]) cb_fx : differential erb codebook, Q13          */
/*   _ (Word16) cb_size :  codebook size                             */
/*   _ (Word16) cb_dim :  codebook dimension                         */
/*   _ (Word16) offset :  index to current segment of erb array      */
/*                        for quantization                           */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*                    _ None                                         */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*   _ (Word16) index: best codebook index                           */
/*-------------------------------------------------------------------*/
/* CALLED FROM : TX                                                  */
/*===================================================================*/

Word16 erb_diff_search_fx(Word16 *prev_erb, const Word16 *curr_erb, Word16 *dif_erb,
                          Word16 *pow_spec, const Word16 *cb_fx,
                          Word16 cb_size, Word16 cb_dim, Word16 offset)
{
    Word16 i, j, mmseindex ;
    Word16 dh, dl;
    Word32 mmse;
    Word32 Ltemp1;
    Word32 Lacc;

    mmse = EVS_LW_MAX;
    move32();
    mmseindex = -1;
    move16();
    FOR (j=0; j<cb_size; j++)
    {

        Lacc = L_deposit_l(0);
        FOR (i=0; i<cb_dim; i++)
        {
            IF (add(cb_fx[j*cb_dim+i],prev_erb[i+offset])<0)
            {
                Ltemp1=L_mult(curr_erb[i+offset],curr_erb[i+offset]); /* Q27 */
                dh=extract_h(Ltemp1);
                dl=extract_l(Ltemp1);
                IF(dl<0)
                {
                    Ltemp1 = L_shl(L_add(65536,dl),14);/* */
                    Ltemp1 = Mult_32_16(Ltemp1,pow_spec[i+offset]);
                    Ltemp1 = L_shl(Ltemp1,1);
                }
                ELSE
                {
                    Ltemp1=(Word32)L_mult0(pow_spec[i+offset],dl);
                }
                Ltemp1 = L_add(L_shr(Ltemp1,15),L_mult(pow_spec[i+offset],dh));
            }
            ELSE
            {
                dh=sub(dif_erb[i+offset],cb_fx[j*cb_dim+i]); /* Q13 */
                Ltemp1=L_mult(dh,dh); /* Q27 */
                dh=extract_h(Ltemp1);
                dl=extract_l(Ltemp1);

                IF(dl<0)
                {
                    Ltemp1 = L_shl(L_add(65536,dl),14);/* */
                    Ltemp1 = Mult_32_16(Ltemp1,pow_spec[i+offset]);
                    Ltemp1 = L_shl(Ltemp1,1);
                }
                ELSE
                {
                    Ltemp1=(Word32)L_mult0(pow_spec[i+offset],dl); /* Q33 */
                }

                Ltemp1=L_add(L_shr(Ltemp1,15),L_mult(pow_spec[i+offset],dh)); /* Q18 */
            }

            IF (sub(cb_fx[j*cb_dim+i],dif_erb[i+offset])<0)
            {
                dh=extract_h(Ltemp1);
                dl=extract_l(Ltemp1);
                IF(dl<0)
                {
                    Ltemp1 = L_shl(L_add(65536,dl),14);/* */
                    Ltemp1 = Mult_32_16(Ltemp1,29491);
                    Ltemp1 = L_shl(Ltemp1,1);
                }
                ELSE
                {
                    Ltemp1=(Word32)L_mult0(29491,dl); /* 29491=0.9 in Q15 */
                }
                Ltemp1=L_add(L_shr(Ltemp1,15),L_mult(dh,29491));
            }
            Lacc=L_add(Lacc,Ltemp1); /* Q18 */
        }

        IF (L_sub(Lacc,mmse)<0)
        {
            mmse = L_add(Lacc, 0);
            mmseindex = j;
            move16();
        }
    }

    return(mmseindex);

}
void Acelp_dec_total_exc(
    Word16 *exc_fx,       /* i/o: adapt. excitation exc         */
    Word16 *exc2_fx,      /* i/o: adapt. excitation/total exc   */
    const Word16 gain_code16,   /* i  : Gain code Q0                  */
    const Word16 gain_pit_fx,   /* i  ; Pitch gain in Q14             */
    const Word16 i_subfr,       /* i  ; subfr                         */
    const Word16 *code_fx       /* i  : code in Q9                   */
)
{
    Word16 i;
    Word32 L_tmp;
    FOR (i = 0; i < L_SUBFR; i++)
    {
        L_tmp = L_shl(L_mult(gain_pit_fx, exc_fx[i+i_subfr]), 1); /*Q16+Q_exc*/
        exc2_fx[i+i_subfr] = round_fx(L_tmp); /*Q_exc*/
        L_tmp = L_add(L_tmp, L_shl(L_mult(gain_code16, code_fx[i]), 6)); /*Q16+Q_exc*/
        exc_fx[i+i_subfr] = round_fx(L_tmp); /*Q_exc*/
    }
}

/*-------------------------------------------------------------------*
 *  UL_inverse
 *
 *  Calculate inverse of UL_val. Output in Q_exp.
 *-------------------------------------------------------------------*/
UWord32 UL_inverse(const UWord32 UL_val, Word16 *exp)
{
    UWord32 UL_tmp;

    *exp = norm_ul(UL_val);
    UL_tmp = UL_lshl(UL_val, *exp);   /* Q32 */

    *exp = add(32, sub(31, *exp));

    return UL_div(0x80000000, UL_tmp);
}

/*-------------------------------------------------------------------*
 *  UL_div
 *
 *  Calculate UL_num/UL_den. UL_num assumed to be Q31, UL_den assumed
 *  to be Q32, then result is in Q32.
 *-------------------------------------------------------------------*/
UWord32 UL_div(const UWord32 UL_num, const UWord32 UL_den)
{
    UWord32 UL_e, UL_Q;
    UWord32 UL_msb, UL_lsb;
    Word16 i;

    UL_e = UL_subNsD(0xffffffff, UL_den);
    UL_Q = UL_num;
    move32();

    FOR (i = 0; i < 5; i++)
    {
        Mpy_32_32_uu(UL_Q, UL_e, &UL_msb, &UL_lsb); /*31+32-32=31 */
        UL_Q = UL_addNsD(UL_Q, UL_msb);
        Mpy_32_32_uu(UL_e, UL_e, &UL_e, &UL_lsb); /*32+32-32=32 */
    }

    return UL_Q;
}

/*-----------------------------------------------------------------------------
 * ratio()
 *
 * Divide the numerator by the denominator.
 *----------------------------------------------------------------------------*/
Word16 ratio(const Word32 numer, const Word32 denom, Word16 *expo)
{
    Word16 expNumer, expDenom;
    Word16 manNumer, manDenom;
    Word16 quotient;

    expDenom = norm_l(denom);                     /* exponent */
    manDenom = extract_h(L_shl(denom, expDenom)); /* mantissa */
    expNumer = norm_l(numer);                     /* exponent */
    manNumer = extract_h(L_shl(numer, expNumer)); /* mantissa */
    manNumer = shr(manNumer, 1); /* Ensure the numerator < the denominator */
    quotient = div_s(manNumer, manDenom); /* in Q14 */

    *expo = sub(expNumer, expDenom);

    return quotient; /* Q14 */
}

/*-----------------------------------------------------------------------*
 * Function hp400_12k8()                                                 *
 *                                                                       *
 * 2nd order Cheb2 high pass filter with cut off frequency at 400 Hz.    *
 * Optimized for fixed-point to get the following frequency response  :  *
 *                                                                       *
 *  frequency  :   0Hz   100Hz  200Hz  300Hz  400Hz  630Hz  1.5kHz  3kHz *
 *  dB loss  :   -infdB  -30dB  -20dB  -10dB  -3dB   +6dB    +1dB    0dB *
 *                                                                       *
 * Algorithm  :                                                          *
 *                                                                       *
 *  y[i] = b[0]*x[i] + b[1]*x[i-1] + b[2]*x[i-2]                         *
 *                   + a[1]*y[i-1] + a[2]*y[i-2];                        *
 *                                                                       *
 *  short b[3] = {3660, -7320,  3660};       in Q12                      *
 *  short a[3] = {4096,  7320, -3540};       in Q12                      *
 *                                                                       *
 *  float -->   b[3] = {0.893554687, -1.787109375,  0.893554687};        *
 *              a[3] = {1.000000000,  1.787109375, -0.864257812};        *
 *-----------------------------------------------------------------------*/
void hp400_12k8_fx(
    Word16 signal[],  /* i/o: input signal / output is divided by 16 */
    const Word16 lg,        /* i  : lenght of signal                       */
    Word16 mem[]      /* i/o: filter memory [6]                      */
)
{
    Word16 i;
    Word16 y1_hi, y1_lo;
    Word32 L_tmp, L_tmp2, L_tmp3;

    y1_hi = mem[2];
    move16();
    y1_lo = mem[3];
    move16();

    L_tmp3 = L_mac(16384L, mem[1], a_hp400_fx[2]); /* rounding to maximize precision */
    L_tmp3 = L_mac(L_tmp3, y1_lo, a_hp400_fx[1]);
    L_tmp3 = L_shr(L_tmp3, 15);
    L_tmp2 = L_mac(L_tmp3, mem[0], a_hp400_fx[2]);
    L_tmp2 = L_mac(L_tmp2, mem[5], b_hp400_fx[2]);
    L_tmp2 = L_mac(L_tmp2, mem[4], b_hp400_fx[1]);
    L_tmp3 = L_mult(mem[4], b_hp400_fx[2]);

    mem[5] = signal[lg-2];

    FOR (i = 1; i < lg; i++)
    {
        /* y[i] = b[0]*x[i] + b[1]*x[i-1] + b[2]*x[i-2] */
        /*      + a[1]*y[i-1] + a[2] * y[i-2] */

        L_tmp = L_mac(L_tmp2, y1_hi, a_hp400_fx[1]);
        L_tmp = L_mac(L_tmp, *signal, b_hp400_fx[0]);

        L_tmp = L_shl(L_tmp, 1);           /* coeff Q12 --> Q13 */

        L_tmp2 = L_mac(L_tmp3, y1_hi, a_hp400_fx[2]);
        L_tmp2 = L_mac(L_tmp2, *signal, b_hp400_fx[1]);
        L_tmp3 = L_mac(16384L, y1_lo, a_hp400_fx[2]); /* rounding to maximize precision */

        y1_lo = L_Extract_lc(L_tmp, &y1_hi);

        L_tmp3 = L_mac(L_tmp3, y1_lo, a_hp400_fx[1]);
        L_tmp3 = L_shr(L_tmp3, 15);

        L_tmp2 = L_add(L_tmp3, L_tmp2);

        L_tmp3 = L_mult(*signal, b_hp400_fx[2]);

        /* signal is divided by 16 to avoid overflow in energy computation */
        *signal++ = round_fx(L_tmp);
    }

    /* y[i] = b[0]*x[i] + b[1]*x[i-1] + b[2]*x[i-2] */
    /*      + a[1]*y[i-1] + a[2] * y[i-2] */

    L_tmp = L_mac(L_tmp2, y1_hi, a_hp400_fx[1]);

    mem[4] = *signal;
    move16();
    L_tmp = L_mac(L_tmp, mem[4], b_hp400_fx[0]);

    L_tmp = L_shl(L_tmp, 1);           /* coeff Q12 --> Q13 */

    mem[0] = y1_hi;
    move16();
    mem[1] = y1_lo;
    move16();
    L_Extract(L_tmp, &mem[2], &mem[3]);

    /* signal is divided by 16 to avoid overflow in energy computation */
    *signal++ = round_fx(L_tmp);

    return;
}


Word16 dot_prod_satcontr(const Word16 *x, const Word16 *y, Word16 qx, Word16 qy, Word16 *qo, Word16 len)
{
    Word16 tmp_tab_x[L_FRAME16k];
    Word16 tmp_tab_y[L_FRAME16k];
    Word16 shift, q, ener, i;
    Word32 L_tmp;
    Word16 *pt1, *pt2;


    Copy( x, tmp_tab_x, len ); /* OPTIMIZE !!!!! the copy into local table is not necessary */
    Copy( y, tmp_tab_y, len ); /* could be reworked to do a 1st iteration with the original x[] and y[] */
    /* then check if there is an overflow and do a more complex 2nd, 3rd, ... processing */
    shift = 0;
    move16();
    DO
    {
        Overflow = 0;
        move16();
        L_tmp = L_shl(1, s_max(sub(add(add(qx, qy), 7), shift), 0));
        pt1 = tmp_tab_x;
        pt2 = tmp_tab_y;
        FOR ( i = 0; i < len; i++ )
        {
            L_tmp = L_mac0(L_tmp, *pt1++, *pt2++); /*Q(qx+qy-shift) */
        }

        IF(Overflow != 0)
        {
            Scale_sig(tmp_tab_x, len, -2);
            Scale_sig(tmp_tab_y, len, -2);
            shift = add(shift, 4);
        }
    }
    WHILE(Overflow != 0);

    q = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, q); /*Q(qx+qy-shift+q) */
    ener = extract_h(L_tmp); /*Q(qx+qy-shift+q-16) */
    q = add(q, add(qx, qy));
    *qo = sub(q, add(shift, 16));

    return ener;
}


/*
 * E_UTIL_f_convolve
 *
 * Parameters:
 *    x           I: input vector <14bits
 *    h           I: impulse response (or second input vector) (1Q14)
 *    y           O: output vetor (result of convolution)
 *
 * Function:
 *    Perform the convolution between two vectors x[] and h[] and
 *    write the result in the vector y[]. All vectors are of length L.
 *    Only the first L samples of the convolution are considered.
 *    Vector size = L_SUBFR
 *
 * Returns:
 *    void
 */
void E_UTIL_f_convolve(const Word16 x[], const Word16 h[], Word16 y[], const Word16 size)
{
    Word16 i, n;
    Word32 L_sum;


    y[0] = mult_r(x[0], h[0]);
    move16();
    FOR (n = 1; n < size; n++)
    {
        L_sum = L_mult(x[0], h[n]);
        FOR (i = 1; i < n; i++)
        {
            L_sum = L_mac(L_sum, x[i], h[n - i]);
        }
        y[n] = mac_r(L_sum, x[i], h[0]);
        move16();

    }
    return;
}

/*-----------------------------------------------------------------------------
 * floating_point_add:
 *
 * Add two floating point numbers: x <- x + y.
 *----------------------------------------------------------------------------*/
void floating_point_add(
    Word32 *mx, /* io: mantissa of the addend Q31 */
    Word16 *ex, /* io: exponent of the addend Q0  */
    const Word32 my,  /* i:  mantissa of the adder Q31  */
    const Word16 ey   /* i:  exponent of the adder Q0   */
)
{
    Word32 accX, accY;
    Word16 align, expo;
    /* NB: This function will not work properly if the mantissa is zero and the exponent is not 32.
       It is up to the caller function to avoid this condition. */
    /* Ensure 1 bit headroom before addition. */
    accX = L_shr(*mx, 1);
    accY = L_shr(my, 1);
    /* First, align the Q-points of the two operands. Then, add. */
    align = sub(*ex, ey);
    test();
    IF (align < 0)
    {
        accX = L_add(accX, L_shl(accY, align));
    }
    ELSE
    {
        accX = L_add(accY, L_shr(accX, align));
        *ex = ey;
        move16();
    }
    /* Normalize the result and update the mantissa and exponent. */
    expo = norm_l(accX);
    *mx = L_shl(accX, expo);
    *ex = sub(add(*ex, expo), 1); /* Subtract 1 due to 1-bit down-shift above ensuring 1 bit headroom before addition. */
    return;
}

