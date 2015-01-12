/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#ifndef __BASOP_UTIL_ROM_H__
#define __BASOP_UTIL_ROM_H__

#include "typedef.h"
#include "basop_util.h"

#define LD_INT_TAB_LEN    120
#define INV_TABLE_SIZE    256
#define SQRT_TABLE_SIZE   256

#ifndef CHEAP_NORM_SIZE
#define CHEAP_NORM_SIZE 161
#endif

#define MINSFTAB  7
#define MAXSFTAB 25

/**
 * \brief  Lookup-Table for binary logarithm
 */
extern const Word16 ldCoeff[7];

/**
  \brief 	Lookup-Table for binary power algorithm
*/
extern const UWord32 exp2_tab_long[32];

/**
  \brief 	Lookup-Table for binary power algorithm
*/
extern const UWord32 exp2w_tab_long[32];

/**
  \brief 	Lookup-Table for binary power algorithm
*/
extern const UWord32 exp2x_tab_long[32];

/**
  \brief 	Lookup-Table for integer binary logarithm
*/
extern const Word32 ldIntCoeff[LD_INT_TAB_LEN];

/**
 * \brief  Lookup-Table for 1/x
*/
extern const Word16 invTable[INV_TABLE_SIZE+1];

/**
 * \brief 1/x, x=[0,1,2,3...]  table
 */
extern const Word16 InvIntTable[65];

/**
 * \brief  Lookup-Table for Squareroot
*/
extern const Word16 sqrtTable[SQRT_TABLE_SIZE+1];
extern const Word16 invSqrtTable[SQRT_TABLE_SIZE+1];

extern const Word32 BASOP_util_normReciprocal[CHEAP_NORM_SIZE];
extern const Word16 f_atan_expand_range[MAXSFTAB-(MINSFTAB-1)];

/**
 * \ brief Sine table
 */
extern const PWord16 SineTable512[257];
extern const PWord16 SineTable480[241];
extern const PWord16 SineTable400[201];
extern const PWord16 SineTable384[193];
extern const PWord16 SineTable640[321];

/**
 * \ brief Lookup for sine tables and windows.
 */
void BASOP_getTables(const PWord16 **ptwiddle, const PWord16 **sin_twiddle, Word16 *sin_step, Word16 length);
const PWord16* getSineWindowTable(Word16 length);

#endif
