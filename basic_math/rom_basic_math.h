#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Decoder static structure               */
#include "stl.h"


/* math_op.c */
extern const Word32 L_table_isqrt[48];

/* table of table_isqrt[i] - table_isqrt[i+1] */
extern const Word16 table_isqrt_diff[48];

extern const Word16 shift_Isqrt_lc[];

extern const Word16 table_pow2[32];

/* table of table_pow2[i+1] - table_pow2[i] */
extern const Word16 table_pow2_diff_x32[32];

extern const Word16 sqrt_table[49];

/* log2.c */
extern const Word32 L_table_Log2_norm_lc[32];
 
extern const Word16 table_diff_Log2_norm_lc[32];
 
extern const Word16 log2_tab[33];
