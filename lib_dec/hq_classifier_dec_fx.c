/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "prot_fx.h"    /* Function prototypes                    */
#include "stl.h"        /* required for wmc_tool                  */

/*--------------------------------------------------------------------------*
 * hq_classifier_dec()
 *
 * HQ mode selector (decision_matrix)
 *--------------------------------------------------------------------------*/

Word16 hq_classifier_dec_fx(         /* o  : Consumed bits                   Q0 */
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure */
    const Word32  core_brate,        /* i  : Core bit rate                   Q0 */
    const Word16 length,             /* i  : Frame length                    Q0 */
    Word16 *is_transient,      /* o  : Transient flag                  Q0 */
    Word16 *hqswb_clas         /* o  : HQ class                        Q0 */
)
{
    Word16 bits;

    test();
    IF ( sub(length, L_FRAME32k) >= 0 && L_sub(core_brate, HQ_32k) <= 0 )
    {
        *hqswb_clas = get_next_indice_fx( st_fx, 2 );
        move16();
        test();
        if (( sub(length, L_FRAME48k) == 0 ) && ( sub(*hqswb_clas, HQ_NORMAL) == 0 ))
        {
            *hqswb_clas = HQ_GEN_FB;
            move16();
        }
    }
    ELSE
    {
        *hqswb_clas = get_next_indice_fx( st_fx, 1 );
        move16();
    }

    *is_transient = 0;
    move16();
    if ( sub(*hqswb_clas, HQ_TRANSIENT) == 0 )
    {
        *is_transient = 1;
        move16();
    }

    test();
    test();
    if ( *hqswb_clas == HQ_NORMAL && sub(length, L_FRAME32k) == 0 && L_sub(core_brate, HQ_32k) <= 0)
    {
        *hqswb_clas = HQ_GEN_SWB;
        move16();
    }

    bits = 1;
    move16();
    test();
    if ( sub(length, L_FRAME32k) >= 0 && L_sub(core_brate, HQ_32k) <= 0 )
    {
        bits = 2;
        move16();
    }

    return bits;
}

