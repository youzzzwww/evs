/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "stl.h"
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */

/*----------------------------------------------------------------------------------*
 * CNG_reset_dec()
 *
 * Reset decoder static variables in case of CNG frame
 *----------------------------------------------------------------------------------*/

void CNG_reset_dec_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure            */
    Word16 *pitch_buf,     /* o  : floating pitch for each subframe   */
    Word16 *voice_factors  /* o  : voicing factors                    */
)
{
    Word16 tmp, exp;
    Word32 L_tmp;
    Copy( UVWB_Ave_fx, st_fx->mem_AR_fx, M );
    set16_fx( st_fx->mem_MA_fx, 0, M );
    /*set16_fx( st_fx->dispMem_fx, 0, 8 );*/
    set16_fx( st_fx->dm_fx.prev_gain_pit , 0, 6 );
    st_fx->dm_fx.prev_gain_code = L_deposit_l(0);
    st_fx->dm_fx.prev_state = 0;
    move16();

    st_fx->tilt_code_fx = 0;
    move16();
    st_fx->gc_threshold_fx = 0;
    move16();

    /* last good received frame for FEC in ACELP */
    st_fx->clas_dec = UNVOICED_CLAS;
    move16();
    st_fx->last_good_fx = UNVOICED_CLAS;
    move16();

    /* LP-filtered pitch gain set to 0 */
    st_fx->lp_gainp_fx = 0;
    move16();

    /* convert CNG energy into CNG gain for ACELP FEC */
    /* st->lp_gainc = sqrt( st->lp_ener ); */
    st_fx->lp_gainc_fx = 0;
    move16();

    IF (st_fx->lp_ener_fx != 0)
    {
        exp = norm_l(st_fx->lp_ener_fx);                            /* In Q6 */
        tmp = extract_h(L_shl(st_fx->lp_ener_fx, exp));
        exp = sub(exp, 30-6);

        tmp = div_s(16384, tmp);
        L_tmp = L_deposit_h(tmp);
        L_tmp = Isqrt_lc(L_tmp, &exp);

        st_fx->lp_gainc_fx = round_fx(L_shl(L_tmp, sub(exp, 12))); /* In Q3 */
    }
    /* reset the pitch buffer in case of FRAME_NO_DATA or SID frames */
    IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
    {
        set16_fx( pitch_buf, L_SUBFR<<6, NB_SUBFR );
    }
    ELSE  /* st->L_frame == L_FRAME16k */
    {
        set16_fx( pitch_buf, L_SUBFR16k<<6, NB_SUBFR16k );
    }

    set16_fx( voice_factors, 32767, NB_SUBFR16k );

    /* deactivate bass post-filter */
    st_fx->bpf_off_fx = 1;
    move16();
    /* Reset active frame counter */
    st_fx->act_cnt2_fx = 0;
    move16();

    return;
}
