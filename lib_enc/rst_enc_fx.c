/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"

/*-------------------------------------------------------------------*
 * CNG_reset_enc()
 *
 * Reset encoder static variables after a CNG frame
 *-------------------------------------------------------------------*/

void CNG_reset_enc_fx(
    Encoder_State_fx *st_fx,        /* i/o: encoder state structure             */
    LPD_state   *mem,        /* i/o: acelp memories                      */
    Word16 *pitch_buf,      /* o  : floating pitch for each subframe    */
    Word16 *voice_factors   /* o  : voicing factors                     */
)
{
    init_gp_clip_fx( st_fx->clip_var_fx );
    Copy( UVWB_Ave_fx, st_fx->mem_AR_fx, M );
    set16_fx( st_fx->mem_MA_fx, 0, M );
    mem->mem_w0 = 0;
    move16();
    mem->tilt_code = 0;
    move16();
    mem->gc_threshold = 0;
    move16();
    Copy( mem->mem_syn2, mem->mem_syn, M );
    /*set16_fx( st_fx->dispMem_fx , 0, 8 ); */
    set16_fx( st_fx->dm_fx.prev_gain_pit , 0, 6 );
    st_fx->dm_fx.prev_gain_code = L_deposit_l(0);
    st_fx->dm_fx.prev_state = 0;
    move16();

    /* last good received frame for FEC in ACELP */
    st_fx->clas_fx = UNVOICED_CLAS;
    move16();

    /* reset the pitch buffer in case of FRAME_NO_DATA or SID frames */
    IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
    {
        set16_fx( pitch_buf, L_SUBFR, NB_SUBFR );
    }
    ELSE  /* st->L_frame == L_FRAME16k */
    {
        set16_fx( pitch_buf, L_SUBFR16k, NB_SUBFR16k );
    }

    set16_fx( voice_factors, 1, NB_SUBFR16k );

    /* deactivate bass post-filter */
    st_fx->bpf_off_fx = 1;
    move16();

    /* Reset active frame counter */
    st_fx->act_cnt2_fx = 0;
    move16();

    return;
}
