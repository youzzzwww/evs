/********************************************************************************
*
*      File             : log2.c
*      Purpose          : Computes log2(L_x)
*
********************************************************************************
*/
 
/*
********************************************************************************
*                         INCLUDE FILES
********************************************************************************
*/
#include "stl.h"
#include "math_op.h"
#include <assert.h>
#include "rom_basic_math.h"

#define LW_SIGN (Word32)0x80000000       /* sign bit */
#define LW_MIN (Word32)0x80000000
#define LW_MAX (Word32)0x7fffffff

#define SW_SIGN (Word16)0x8000          /* sign bit for Word16 type */
#define SW_MIN (Word16)0x8000           /* smallest Ram */
#define SW_MAX (Word16)0x7fff           /* largest Ram */

 
/*
********************************************************************************
*                         PUBLIC PROGRAM CODE
********************************************************************************
*/

/*************************************************************************
 *
 *   FUNCTION:   Log2_norm_lc()
 *
 *   PURPOSE:   Computes log2(L_x, exp),  where   L_x is positive and
 *              normalized, and exp is the normalisation exponent
 *              If L_x is negative or zero, the result is 0.
 *
 *   DESCRIPTION:
 *        The function Log2(L_x) is approximated by a table and linear
 *        interpolation. The following steps are used to compute Log2(L_x)
 *
 *           1- exponent = 30-norm_exponent
 *           2- i = bit25-b31 of L_x;  32<=i<=63  (because of normalization).
 *           3- a = bit10-b24
 *           4- i -=32
 *           5- fraction = table[i]<<16 - (table[i] - table[i+1]) * a * 2
 *
 *************************************************************************/
Word16 Log2_norm_lc (   /* (o) : Fractional part of Log2. (range: 0<=val<1)  */
    Word32 L_x          /* (i) : input value (normalized)                    */
)
{
    Word16 i, a;
    Word16 y;

    if (L_x <= 0)
        L_x = L_deposit_h(0x4000);

    L_x = L_shr (L_x, 9);
    a = extract_l (L_x);                      /* Extract b10-b24 of fraction */
    a = lshr(a, 1);

    i = mac_r(L_x, -32*2-1, 16384);           /* Extract b25-b31 minus 32 */

    y = mac_r(L_table_Log2_norm_lc[i], table_diff_Log2_norm_lc[i], a);  /* table[i] << 16 - diff*a*2 */

    return y;
}

/*******************************************************/
/****Added by Sasken for disabling 40 bit operations ****/
/********************************************************/

Word32 log10_fx(Word32 Linput)
{
  Word16 n1, frac, p1, p2, q1;
  Word32 Ltemp1, Ltemp2;
  Word32 L_tmp;

  if (Linput<=0) return(LW_MIN);
  n1=norm_l(Linput);
  Ltemp1=(Word32)L_shl(Linput,n1);
  
  Ltemp2=L_mult(extract_h(Ltemp1),0x40);
  frac=extract_l(Ltemp2);

  p1=log2_tab[sub(extract_h(Ltemp2),0x20)];
  p2=log2_tab[sub(extract_h(Ltemp2),0x1F)];
  Ltemp2=L_mult(n1,0x200);
  n1=extract_l(Ltemp2);
  
  Ltemp1=L_add(L_deposit_h(p1),0x8000); /* Add rounding bit */

  IF(frac >= 0)
  {
      Ltemp1=L_sub(Ltemp1,(Word32)L_mult0(p1,frac));
      Ltemp1=L_add(Ltemp1,(Word32)L_mult0(p2,frac));
  }
  ELSE
  {
      L_tmp = L_add(65536,frac);
      L_tmp = L_tmp*p1;
      Ltemp1=L_sub(Ltemp1,L_tmp);

      L_tmp = L_add(65536,frac);
      L_tmp = L_tmp*p2;
      Ltemp1=L_add(Ltemp1,L_tmp);
  }
  q1=extract_h(Ltemp1);
  Ltemp1=L_mult(q1,0x6054);
  Ltemp1=L_msu(Ltemp1,0x6054,n1);
  return(L_shr(Ltemp1,1));
      
}

Word32 pow_10(Word32 x , Word16 *Q)
{
  Word16 xl,xh, t1, t2, n;
  Word32 Ltemp1;
  Word32 Lacc;
  Word32 L_tmp;
  Word16 n1,i;
  Word16 count = 0;

  move16();;

  xl=extract_l(x);
  xh=extract_h(x);
  
  IF(xl < 0)
  {
      L_tmp = L_add(65536,xl);
      Ltemp1=(Word32) (0x6a4d*L_tmp );
  }
  ELSE
  {
      Ltemp1=L_mult0(0x6a4d,xl);
  }
  Ltemp1=L_add(L_shr(Ltemp1,16),L_shr(L_mult(xh,0x6a4d),1));
  
  
  Lacc=L_sub(-1L, Ltemp1); /* Lacc=~Lacc, 1's complement */
  t1=extract_l(L_shr(Lacc,7));
  
  Ltemp1=L_shr(Ltemp1,7);
  n1 = extract_h(Ltemp1);
  n=sub(n1,14);
  *Q = 14;    move16();
  IF(t1<0)
  {
      L_tmp = L_add(65536,t1);
      t2=extract_h(L_tmp*L_tmp);
  }
  ELSE
  {
      t2=extract_h(L_mult0(t1,t1));
  }
  
  Lacc = L_deposit_h(0x1FEF);
  IF(t2 < 0)
  {
      L_tmp = L_add(65536,t2);
      Lacc = L_add(Lacc,(Word32)(L_tmp*0x057C));
  }
  ELSE
  {
      Lacc = L_add(Lacc,(Word32)L_mult0(t2,0x057C));
  }

  IF(t1 < 0)
  {
      L_tmp = L_add(65536,t1);
      Lacc = L_sub(Lacc,(Word32)(L_tmp*0x155C));
  }
  ELSE
  {
      Lacc = L_sub(Lacc,(Word32)L_mult0(t1,0x155C));
  }

  L_tmp = Lacc;
  FOR(i =1 ;i <= n ;i++)
  {
	  Overflow = 0;   move16();
	  L_tmp = L_shl(L_tmp,i);
	  IF(Overflow)
      {
	      count = add(count,1);
      }
  }
  *Q = sub(*Q,count); move16();

  return(L_shl(Lacc,sub(n,count)));

}

Word16 Log2_lc(         /* (o) : Fractional part of Log2. (range: 0<=val<1)  */
    Word32 L_x,         /* (i) : input value                                 */
    Word16 *exponent    /* (o) : Integer part of Log2.   (range: 0<=val<=30) */
)
{
    Word16 exp;

    if (L_x <= 0)
        L_x = L_deposit_l(0x1);

    exp = norm_l (L_x);
    *exponent = sub(30, exp); move16(); 

    return Log2_norm_lc(L_shl(L_x, exp));
}
