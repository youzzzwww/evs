/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <assert.h>
#include "basop_util.h"
#include "prot_fx.h"
#include "stl.h"

#define  PI_HALF_0Q15	51472		/* ~=round(pi/2*2^15) */
#define  PI2_15Q16		0x0006487F	/* ~=round(2*PI*2^16) */
#define  PI2_10Q21		13176795	/* ~=round(2*PI*2^21) */
#define  PI2_11Q20    6588397	/* ~=round(2*PI*2^20) */
#define  P54_1Q14		 8847		/* ~=round(0.54*2^14) */
#define  P54_0Q15		17695		/* ~=round(0.54*2^15) */
#define	 P46_0Q15		15073		/* ~=round(0.46*2^15) */
#define	 P92_0Q15		30147		/* ~=round(0.92*2^15) */
#include "options.h"
#include "rom_basop_util.h"

/* precalculated window-types for the EVS configs */
extern const Word16 ham_cos_window_256_128[384];
extern const Word16 ham_cos_window_320_160[480];


void ham_cos_window(Word16 *fh,	/* o: 0Q15 */
                    const Word16 n1,	/* i:      */
                    const Word16 n2   /* i:      */
                   )
{
    Word16  i;
    Word32  cte, cc;



    test();
    test();
    IF(sub(n1, 256) == 0 && sub(n2, 128) == 0)
    {
        Copy(ham_cos_window_256_128, fh, 256 + 128);
    }
    ELSE IF(sub(n1, 320) == 0 && sub(n2, 160) == 0)
    {
        Copy(ham_cos_window_320_160, fh, 320 + 160);
    }
    ELSE
    {

        assert( n1>=102 ); /* if n1 is too low -> overflow in div_l */
        /*	cte = PI2/(Float32)(2*n1 - 1);	*/
        BASOP_SATURATE_WARNING_OFF
        move16();
        cte = L_deposit_l(div_l(PI2_10Q21,sub(shl(n1,1),1)));				/*0Q15*/
        BASOP_SATURATE_WARNING_ON
        cc = 0;
        FOR (i = 0; i < n1; i++)
        {
            /* fh_f[i] = 0.54f - 0.46f * (Float32)cos(cc);	*/
            BASOP_SATURATE_WARNING_OFF
            fh[i] = sub(P54_0Q15, mult_r(getCosWord16(round_fx(L_shl(cc,9))),P92_0Q15));		/*0Q15*/	move16();
            BASOP_SATURATE_WARNING_ON
            cc = L_add(cc, cte);											/*0Q15*/
        }

        assert( n2>=26 ); /* if n2 is too low -> overflow in div_l */
        /* cte = PI2/(Float32)(4*n2 - 1);	*/
        cte = L_deposit_l(div_l(PI2_11Q20,sub(shl(n2,2),1)));				/*0Q15*/
        cc = 0;
        move16();

        add(n1,n2);
        BASOP_SATURATE_WARNING_OFF
        FOR (i = n1; i < n1+n2; i++)
        {
            /* fh_f[i] = (Float32)cos(cc);		*/
            fh[i] = shl(getCosWord16(round_fx(L_shl(cc,10))),1);					/*0Q15*/ move16();
            cc = L_add(cc, cte);
        }
        BASOP_SATURATE_WARNING_ON

    }


    return;
}

