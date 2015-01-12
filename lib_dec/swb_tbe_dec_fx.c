/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/



#include <stdlib.h>
#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "rom_dec_fx.h"
#include "stl.h"

/*-----------------------------------------------------------------*
 * Local functions
 *-----------------------------------------------------------------*/


static void dequantizeSHBparams_fx_9_1( Decoder_State_fx* st_fx, const Word16 extl, Word32 extl_brate,
                                        Word16* Q_lsf, Word16* Q_subgain, Word32* Q_framegrain, Word16* uv_flag,
                                        Word32* Q_shb_ener_sf_32, Word16* Q_shb_res_gshape, Word16* Q_mixFactors);


/*static void MSdecode_fx( const Word16 index1, const Word16 index2, Word16* reconst, const Word16 n_cols,*/
/*                            const Word16* codebook1, const Word16* codebook2, const Word16 par_ser );*/
static void find_max_mem_dec( Decoder_State_fx* st_fx, Word16* n_mem, Word16 *n_mem2
                              , Word16 *n_mem3
                            );
static void rescale_genSHB_mem_dec( Decoder_State_fx* st_fx, Word16 sf );
static void find_max_mem_wb( Decoder_State_fx* st_fx, Word16* n_mem );
static void rescale_genWB_mem( Decoder_State_fx* st_fx, Word16 sf );
static void Dequant_lower_LSF_fx( const Word16 lsf_idx[], Word16 lsf_q[] ,const Word16 rf_mode_SWB1K2);
static void Map_higher_LSF_fx( Word16 lsf_q[], const Word16 m, const Word16 grid_in[] );
static void Dequant_mirror_point_fx( const Word16 lsf_q[], const Word16 m_idx, Word16* m );

/* gain shape concealment code */
static void gradientGainShape(Decoder_State_fx *st_fx, Word16 *GainShape, Word32 *GainFrame);

/*-------------------------------------------------------------------*
 * find_max_mem_dec()
 *
 * Find norm and max in TBE memories and past buffers
 *-------------------------------------------------------------------*/
void find_max_mem_dec(
    Decoder_State_fx *st_fx,
    Word16 *n_mem,
    Word16 *n_mem2
    ,Word16 *n_mem3
)
{
    Word16 i;
    Word16 n_mem_32;
    Word16 max  = 0;
    Word32 Lmax = 0;
    Word16 tempQ15, max2 = 0;

    Word16 max3;
    Word32 tempQ32, Lmax3;

    /* old BWE exc max */
    FOR( i = 0; i < NL_BUFF_OFFSET; i++ )
    {
        tempQ15 = abs_s( st_fx->old_bwe_exc_extended_fx[i] );
        max = s_max( max, tempQ15 );
    }

    /* decimate all-pass steep memory */
    FOR ( i = 0; i < 7; i++ )
    {
        tempQ15 = abs_s( st_fx->mem_genSHBexc_filt_down_shb_fx[i] );
        max = s_max(max, tempQ15);
    }

    /*  -- keep norm of state_lpc_syn_fx, state_syn_shbexc_fx,
           and mem_stp_swb_fx separately for 24.4 and 32kbps ----*/
    /* findMaxMem2() inside tbe com */
    FOR ( i = 0; i < LPC_SHB_ORDER; i++ )
    {
        tempQ15 = abs_s( st_fx->state_lpc_syn_fx[i] );
        max2 = s_max(max2, tempQ15);
    }

    /* findMaxMem2() inside tbe com */
    FOR ( i = 0; i < L_SHB_LAHEAD; i++ )
    {
        tempQ15 = abs_s( st_fx->state_syn_shbexc_fx[i] );
        max2 = s_max(max2, tempQ15);
    }

    /* findMaxMem2() inside tbe com */
    FOR ( i = 0; i < LPC_SHB_ORDER; i++ )
    {
        tempQ15 = abs_s( st_fx->mem_stp_swb_fx[i] );
        max2 = s_max(max2, tempQ15);
    }

    /* for total_brate > 16.4kbps, use n_mem2; else account for the max2 for n_mem calculation */
    *n_mem2 = norm_s(max2);
    if(max2 == 0) *n_mem2 = 15;

    if(L_sub(st_fx->total_brate_fx, ACELP_24k40) < 0)
    {
        max = s_max(max, max2);
    }

    /* de-emph and pre-emph memory */
    tempQ15 = abs_s( st_fx->tbe_demph_fx );
    max = s_max(max, tempQ15);

    tempQ15 = abs_s( st_fx->tbe_premph_fx );
    max = s_max(max, tempQ15);

    /* estimate the norm for 16-bit memories  */
    *n_mem = norm_s( max );
    if( max == 0 )
    {
        *n_mem = 15;
    }

    /* estimate the norm for 32-bit memories */
    Lmax = L_abs( st_fx->mem_csfilt_fx[0] );  /* only element [0] is used in env. shaping */

    n_mem_32 = norm_l( Lmax );
    if( Lmax == 0 )
    {
        n_mem_32 = 31;
    }

    tempQ15 = sub( s_min( *n_mem, n_mem_32 ), 1 );
    *n_mem = s_max( tempQ15, 0 );

    /* --------------------------------------------------------------*/
    /* Find headroom for synthesis stage associated with these memories:
          1. st_fx->syn_overlap_fx
          2. st_fx->genSHBsynth_state_lsyn_filt_shb_local
          3. st_fx->genSHBsynth_Hilbert_Mem_fx (32-bit) */
    max3 = 0;
    /* find max in prev overlapSyn */
    FOR ( i = 0; i < L_SHB_LAHEAD; i++ )
    {
        tempQ15 = abs_s( st_fx->syn_overlap_fx[i] );
        max3 = s_max(max3, tempQ15);
    }
    /* find max in prev genSHBsynth_state_lsyn_filt_shb_local_fx */
    FOR ( i = 0; i < (2*ALLPASSSECTIONS_STEEP+1); i++ )
    {
        tempQ15 = abs_s( st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx[i] );
        max3 = s_max(max3, tempQ15);
    }
    /* find max in prev int_3_over_2_tbemem_dec_fx */
    IF( L_sub(st_fx->output_Fs_fx, 48000) == 0 )
    {
        FOR ( i = 0; i < INTERP_3_2_MEM_LEN; i++ )
        {
            tempQ15 = abs_s( st_fx->int_3_over_2_tbemem_dec_fx[i] );
            max3 = s_max(max3, tempQ15);
        }
    }
    /* estimate the norm for 16-bit memories  */
    *n_mem3 = norm_s( max3 );
    if( max3 == 0 ) *n_mem3 = 15;

    Lmax3 = 0;
    IF(sub(st_fx->L_frame_fx, L_FRAME) == 0)
    {
        /* find max in prev genSHBsynth_Hilbert_Mem_fx */
        FOR ( i = 0; i < HILBERT_MEM_SIZE; i++ )
        {
            tempQ32 = L_abs( st_fx->genSHBsynth_Hilbert_Mem_fx[i] );
            Lmax3 = L_max(Lmax3, tempQ32);
        }
    }

    /* estimate the norm for 32-bit memories  */
    n_mem_32 = norm_l( Lmax3 );
    if( Lmax3 == 0 )  n_mem_32 = 31;

    tempQ15 = sub( s_min( *n_mem3, n_mem_32 ), 2 );   /* very important leave at least 2 bit head room
                                                         because of the Hilber transform and Q14 coeffs */
    *n_mem3 = s_max( tempQ15, 0 );
    /* --------------------------------------------------------------*/
}

/*-------------------------------------------------------------------*
 * rescale_genSHB_mem_dec()
 *
 * Rescale genSHB memories
 *-------------------------------------------------------------------*/
void rescale_genSHB_mem_dec(
    Decoder_State_fx *st_fx,
    Word16 sf )
{
    Word16 i;

    FOR( i = 0; i < NL_BUFF_OFFSET; i++ )
    {
        st_fx->old_bwe_exc_extended_fx[i] = shl( st_fx->old_bwe_exc_extended_fx[i], sf );
        move16();
    }

    FOR( i = 0; i < 7; i++ )
    {
        st_fx->mem_genSHBexc_filt_down_shb_fx[i] = shl( st_fx->mem_genSHBexc_filt_down_shb_fx[i], sf );
        move16();
    }

    /*  -- Apply memory scaling for 13.2 and 16.4k bps using sf ----*/
    IF(L_sub(st_fx->total_brate_fx, ACELP_24k40) < 0)
    {
        FOR ( i = 0; i < LPC_SHB_ORDER; i++ )
        {
            st_fx->state_lpc_syn_fx[i] = shl( st_fx->state_lpc_syn_fx[i], sf );
            move16();
        }

        FOR ( i = 0; i < L_SHB_LAHEAD; i++ )
        {
            st_fx->state_syn_shbexc_fx[i] = shl( st_fx->state_syn_shbexc_fx[i], sf );
            move16();
        }
    }

    st_fx->mem_csfilt_fx[0] = L_shl( st_fx->mem_csfilt_fx[0], sf );
    move32();

    st_fx->tbe_demph_fx = shl_r( st_fx->tbe_demph_fx, sf );
    move16();
    st_fx->tbe_premph_fx = shl_r( st_fx->tbe_premph_fx, sf );
    move16();

}

void find_max_mem_wb( Decoder_State_fx* st_fx, Word16* n_mem )
{
    Word16 i;
    Word16 max  = 0;
    Word32 Lmax = 0;
    Word16 n_mem_32;

    FOR( i = 0; i < NL_BUFF_OFFSET; i++ )
    max = s_max( max, abs_s( st_fx->old_bwe_exc_extended_fx[i] ) );

    FOR( i = 0; i < 7; i++ )
    {

        if( abs_s( st_fx->mem_genSHBexc_filt_down_shb_fx[i] ) > max )
            max = abs_s( st_fx->mem_genSHBexc_filt_down_shb_fx[i] );
    }

    FOR( i = 0; i < 7; i++ )
    {
        if( abs_s( st_fx->mem_genSHBexc_filt_down_wb2_fx[i] ) > max )
            max = abs_s( st_fx->mem_genSHBexc_filt_down_wb2_fx[i] );
    }

    FOR( i = 0; i < 7; i++ )
    {

        if( abs_s( st_fx->mem_genSHBexc_filt_down_wb3_fx[i] ) > max )
            max = abs_s( st_fx->mem_genSHBexc_filt_down_wb3_fx[i] );
    }

    FOR( i = 0; i < 10; i++ )
    {

        if( abs_s( st_fx->state_lpc_syn_fx[i] ) > max )
            max = abs_s( st_fx->state_lpc_syn_fx[i] );
    }

    FOR( i = 0; i < 5; i++ )
    {

        if( abs_s( st_fx->state_syn_shbexc_fx[i] ) > max )
            max = abs_s( st_fx->state_syn_shbexc_fx[i] );
    }

    IF ( max == 0 )
    {
        *n_mem = 15;
        move16();
    }
    ELSE
    {
        *n_mem = norm_s( max );
        move16();
    }


    FOR( i = 0; i < 2; i++ )
    {

        if( L_abs( st_fx->mem_csfilt_fx[i] ) > Lmax )
            Lmax = L_abs( st_fx->mem_csfilt_fx[i] );
    }

    IF ( Lmax == 0 )
    {
        n_mem_32 = 31;
        move16();
    }
    ELSE
    {
        n_mem_32 = norm_l( Lmax );
    }

    *n_mem = sub( s_min( *n_mem, n_mem_32 ), 1 );
    *n_mem = s_max( *n_mem, 0 );
}

void rescale_genWB_mem( Decoder_State_fx* st_fx, Word16 sf )
{
    Word16 i;
    FOR( i = 0; i < NL_BUFF_OFFSET; i++ )
    {
        st_fx->old_bwe_exc_extended_fx[i] = shl( st_fx->old_bwe_exc_extended_fx[i], sf );
        move16();
    }

    FOR( i = 0; i < 10; i++ )
    {
        st_fx->state_lpc_syn_fx[i] = shl( st_fx->state_lpc_syn_fx[i], sf );
        move16();
    }

    FOR( i = 0; i < 5; i++ )
    {
        st_fx->state_syn_shbexc_fx[i] = shl( st_fx->state_syn_shbexc_fx[i], sf );
        move16();
    }

    FOR( i = 0; i < 7; i++ )
    {
        st_fx->mem_genSHBexc_filt_down_shb_fx[i] = shl( st_fx->mem_genSHBexc_filt_down_shb_fx[i], sf );
        move16();
    }

    FOR( i = 0; i < 7; i++ )
    {
        st_fx->mem_genSHBexc_filt_down_wb2_fx[i] = shl( st_fx->mem_genSHBexc_filt_down_wb2_fx[i], sf );
        move16();
    }

    FOR( i = 0; i < 7; i++ )
    {
        st_fx->mem_genSHBexc_filt_down_wb3_fx[i] = shl( st_fx->mem_genSHBexc_filt_down_wb3_fx[i], sf );
        move16();
    }

    FOR( i = 0; i < 2; i++ )
    {
        st_fx->mem_csfilt_fx[i] = L_shl( st_fx->mem_csfilt_fx[i], sf );
        move32();
    }
}



void InitSWBdecBuffer_fx(
    Decoder_State_fx* st_fx /* i/o: SHB decoder structure */
)
{
    set16_fx( st_fx->old_bwe_exc_fx, 0, ( PIT16k_MAX * 2 ) );
    st_fx->bwe_seed_fx[0] = 23;
    move16();
    st_fx->bwe_seed_fx[1] = 59;
    move16();

    set16_fx( st_fx->old_bwe_exc_extended_fx, 0, NL_BUFF_OFFSET );
    st_fx->bwe_non_lin_prev_scale_fx = 0;
    move16();
    st_fx->last_voice_factor_fx = 0;
    move16();

    set32_fx( st_fx->genSHBsynth_Hilbert_Mem_fx, 0, HILBERT_MEM_SIZE );
    set16_fx(st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx, 0, (2*ALLPASSSECTIONS_STEEP+1));  /* Interp all pass memory */

    st_fx->syn_dm_phase_fx = 0;
    move16();
    st_fx->prev_fbbwe_ratio_fx = FL2WORD16(1.0f);

    /* these are fd-bwe constants */
    st_fx->prev_wb_bwe_frame_pow_fx = FL2WORD32_SCALE(0.001f, 31-22); /* Q22 */
    st_fx->prev_swb_bwe_frame_pow_fx = FL2WORD32_SCALE(0.001f, 31-22); /* Q22 */
    st_fx->prev_fb_ener_adjust_fx = 0;

    set16_fx( st_fx->prev_lpc_wb_fx, 0, LPC_SHB_ORDER_WB);
    st_fx->prev_Q_bwe_exc = 31;
    move16();
    st_fx->prev_ener_fx_Q = 31;
    move16();
    st_fx->prev_Qx = 0;
    move16();
    st_fx->prev_frame_pow_exp = 0;
    move16();
    st_fx->prev_Q_bwe_syn = 0;
    move16();
    st_fx->prev_Q_bwe_syn2 = 0;
    move16();
    return;
}


void ResetSHBbuffer_Dec_fx( Decoder_State_fx* st_fx /* i/o: SHB encoder structure */ )
{
    Word16 i;
    Word16 f;
    Word16 inc;

    IF( st_fx->extl_fx != WB_TBE )
    {
        f = 1489;
        move16();     /* Q15 */
        inc = 1489;
        move16();      /* Q15 */
    }
    ELSE
    {
        f = 5461;
        move16();/* Q15 */
        inc = 5461;
        move16(); /* Q15 */
    }

    /* states for the filters used in generating SHB excitation from WB excitation*/
    set32_fx( st_fx->mem_csfilt_fx, 0, 2 );

    /* states for the filters used in generating SHB signal from SHB excitation*/
    set16_fx( st_fx->state_syn_shbexc_fx, 0, L_SHB_LAHEAD );
    set16_fx( st_fx->state_lpc_syn_fx, 0, LPC_SHB_ORDER );

    /* states for the filters used in generating SHB signal from SHB excitation in wideband*/
    set16_fx( st_fx->mem_genSHBexc_filt_down_shb_fx, 0, 2*ALLPASSSECTIONS_STEEP+1);
    set16_fx( st_fx->mem_genSHBexc_filt_down_wb2_fx, 0, 2*ALLPASSSECTIONS_STEEP+1);
    set16_fx( st_fx->mem_genSHBexc_filt_down_wb3_fx, 0, 2*ALLPASSSECTIONS_STEEP+1);
    set16_fx( st_fx->state_lsyn_filt_shb_fx,0, 2 * L_FILT16k );
    set16_fx( st_fx->state_lsyn_filt_dwn_shb_fx,0, 2 * L_FILT16k );
    set16_fx( st_fx->state_32and48k_WB_upsample_fx, 0, 2 * L_FILT16k );

    /* States for the local synthesis filters */
    set16_fx( st_fx->syn_overlap_fx, 0, L_SHB_LAHEAD );

    /* States for FEC */

    IF( st_fx->extl_fx != WB_TBE )
    {
        FOR( i = 0; i < LPC_SHB_ORDER; i++ )
        {
            st_fx->lsp_prevfrm_fx[i] = f;
            move16(); /*Q15*/
            f = add( f, inc );
            move16();
        }
    }
    ELSE
    {
        FOR( i = 0; i < LPC_SHB_ORDER_WB; i++ )
        {
            st_fx->lsp_prevfrm_fx[i] = f;
            move16();/*Q15*/
            f = add( f, inc );
            move16();
        }
    }
    st_fx->GainFrame_prevfrm_fx = 0;
    move16();/*Q18*/
    st_fx->GainAttn_fx = 32767;
    move16();/*Q15*/
    st_fx->tbe_demph_fx = 0;
    st_fx->tbe_premph_fx = 0;
    set16_fx(st_fx->mem_stp_swb_fx, 0, LPC_SHB_ORDER);
    st_fx->gain_prec_swb_fx = 16384;/*Q14 =1*/
    set16_fx( &st_fx->GainShape_Delay[0], 0, NUM_SHB_SUBFR / 2 );

    set16_fx(st_fx->old_core_synth_fx, 0, L_FRAME16k);
    set16_fx(st_fx->old_tbe_synth_fx, 0, L_FRAME48k);

    return;
}




/*==========================================================================*/
/* FUNCTION      : void wb_tbe_dec_fx ()                                    */
/*--------------------------------------------------------------------------*/
/* PURPOSE       : WB TBE decoder, 6 - 8 kHz band decoding module           */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                       */
/* _Word16 coder_type             i  : coding type                          */
/* _Word32 *bwe_exc_extended      i  : bandwidth extended exciatation 2*Q_exc*/
/* _Word16 Q_exc                  i  : Q format                             */
/* _Word16 voice_factors[]        i  : voicing factors          Q15         */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                       */
/* _Word16 *synth               o  : WB synthesis/final synthesis  Q_synth  */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                 */
/* Decoder_State_fx *st_fx,       i/o: decoder state structure              */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                       */
/*           _ None                                                         */
/*--------------------------------------------------------------------------*/
/* CALLED FROM :                                                            */
/*==========================================================================*/
void wb_tbe_dec_fx(
    Decoder_State_fx* st_fx,    /* i/o: decoder state structure         */
    const   Word16 coder_type,          /* i  : coding type                     */
    Word32* bwe_exc_extended,   /* i  : bandwidth extended exciatation 2*Q_exc*/
    const   Word16 Q_exc,
    const   Word16 voice_factors[],     /* i  : voicing factors                 */
    Word16* synth,              /* o  : WB synthesis/final synthesis    */
    Word16* Q_synth
)
{
    Word16 i;
    Word16 shaped_wb_excitation [ ( L_FRAME16k + L_SHB_LAHEAD )/4 ];
    Word16 shaped_wb_excitation_frac[ L_FRAME16k/4 ];
    Word16 bwe_exc_extended_16[ L_FRAME32k+40 ];
    Word16 exc4kWhtnd [ L_FRAME16k / 4];
    Word16 lsf_wb[ LPC_SHB_ORDER_WB ], lpc_wb[ LPC_SHB_ORDER_WB + 1 ], GainShape[ NUM_SHB_SUBFR ];
    Word32 GainFrame;
    Word16 error[ L_FRAME16k ];
    Word16 synth_frac[ L_FRAME16k ];
    Word16 upsampled_synth[ L_FRAME48k ];
    Word32 prev_pow, curr_pow, curr_frame_pow;
    Word16 curr_frame_pow_exp;
    Word16 temp, scale, n;
    Word16 j;

    Word16 Q_bwe_exc, Q_bwe_exc_ext, Qx;
    Word16 n_mem, cnt;
    Word16 max  = 0;
    Word32 L_tmp, Lacc, Lscale, Lmax = 0;
    Word16 tmp, exp, sc;
    Word16 vf_modified[ NB_SUBFR16k ];
    Word16 uv_flag = 0;
    Word16 dummy=0;
    Word32 dummy2[HILBERT_MEM_SIZE]= {0};
    Word16 f, inc;

    IF( st_fx->bws_cnt_fx == 0 )
    {
        /* Initialization */
        set16_fx(GainShape, FL2WORD16(0.35f), NUM_SHB_SUBFR);
        GainFrame = 1;

        IF( !st_fx->bfi_fx )
        {
            IF(sub(st_fx->use_partial_copy,1)==0)
            {
                Copy( st_fx->lsp_prevfrm_fx, lsf_wb, LPC_SHB_ORDER_LBR_WB );
                set16_fx( GainShape, RECIP_ROOT_EIGHT_FX, NUM_SHB_SUBFR/2 );

                IF( sub(st_fx->rf_frame_type,RF_NELP) == 0 )
                {
                    /* Frame gain */
                    Copy32( SHBCB_FrameGain16_fx + st_fx->rf_indx_tbeGainFr[0], &GainFrame, 1 );
                    IF( sub(st_fx->core_fx,ACELP_CORE) == 0 && sub(st_fx->last_core_fx,ACELP_CORE) == 0
                        && !st_fx->prev_use_partial_copy && sub(st_fx->prev_coder_type_fx,UNVOICED) == 0
                        && L_sub(GainFrame,st_fx->GainFrame_prevfrm_fx) != 0 )
                    {
                        /*GainFrame = 0.2f*GainFrame + 0.8f*st_fx->GainFrame_prevfrm_fx;*/
                        GainFrame = L_add(Mult_32_16(st_fx->GainFrame_prevfrm_fx, 26214), Mult_32_16(GainFrame, 6553));
                    }
                }
                ELSE
                {
                    temp = 0;
                    move16();
                    /* Frame gain */
                    SWITCH (st_fx->rf_indx_tbeGainFr[0])
                    {
                    case 0:
                        GainFrame = 131072; /* 0.5f in Q18 */
                        IF(L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(1.25, 31-18)) <= 0) temp = FL2WORD16(0.8);
                        move16();
                        BREAK;
                    case 1:
                        GainFrame = 524288; /* 2.0f in Q18 */
                        IF(L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(1.25, 31-18)) > 0 && L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(3, 31-18)) <= 0) temp = FL2WORD16(0.8);
                        move16();
                        test();
                        BREAK;
                    case 2:
                        GainFrame = 1048576;/* 4.0f in Q18 */
                        IF(L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(3, 31-18)) > 0 && L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(6, 31-18)) <= 0) temp = FL2WORD16(0.8);
                        move16();
                        test();
                        BREAK;
                    case 3:
                        GainFrame = 2097152;/* 8.0f in Q18 */
                        IF(L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(6, 31-18)) > 0 && L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(16, 31-18)) <= 0) temp = FL2WORD16(0.8);
                        move16();
                        test();
                        BREAK;
                    default:
                        fprintf(stderr, "RF SWB-TBE gain bits not supported.");
                    }
                    GainFrame = L_add(Mult_32_16(st_fx->GainFrame_prevfrm_fx, temp), Mult_32_16(GainFrame, sub(32767,temp)));
                    IF(sub(st_fx->core_fx,ACELP_CORE) == 0 && sub(st_fx->last_core_fx,ACELP_CORE) == 0)
                    {
                        IF(!st_fx->prev_use_partial_copy && sub(st_fx->last_coder_type_fx, VOICED) == 0 && sub(st_fx->rf_frame_type,RF_GENPRED) == 0
                        && sub(st_fx->prev_tilt_code_dec_fx,1497) < 0 && sub(st_fx->prev_tilt_code_dec_fx,200) > 0 )
                        {
                            GainFrame = Mult_32_16(GainFrame,9830);
                        }
                    }
                }
            }
            ELSE
            {
                /* de-quantization */
                dequantizeSHBparams_fx_9_1( st_fx, st_fx->extl_fx, st_fx->extl_brate_fx, lsf_wb, GainShape, &GainFrame, &uv_flag, 0, 0, 0 );
            }
        }
        ELSE
        {
            IF( L_sub( st_fx->extl_brate_fx, WB_TBE_0k35) == 0 )
            {
                Copy( st_fx->lsp_prevfrm_fx, lsf_wb, LPC_SHB_ORDER_LBR_WB );
            }
            ELSE
            {
                Copy( st_fx->lsp_prevfrm_fx, lsf_wb, LPC_SHB_ORDER_WB );
            }
            set16_fx( GainShape, RECIP_ROOT_EIGHT_FX, NUM_SHB_SUBFR / 2 );

            st_fx->GainAttn_fx = mult_r( st_fx->GainAttn_fx, 27853 );
            move16();

            IF(sub(st_fx->codec_mode, MODE1) == 0)
            {
                GainFrame = Mult_32_16( st_fx->GainFrame_prevfrm_fx, st_fx->GainAttn_fx ); /*Q18*/
            }
            ELSE
            {
                GainFrame = st_fx->GainFrame_prevfrm_fx; /*Q18*/
            }
        }

        IF( st_fx->extl_brate_fx == WB_TBE_0k35 )
        {
            /* convert LSPs back into LP coeffs */
            lsp2lpc_fx( lpc_wb + 1, lsf_wb, st_fx->prev_lpc_wb_fx, LPC_SHB_ORDER_LBR_WB );
            set16_fx( lpc_wb + LPC_SHB_ORDER_LBR_WB + 1, 0, ( LPC_SHB_ORDER_WB - LPC_SHB_ORDER_LBR_WB ) );
            FOR( i = 0; i < LPC_SHB_ORDER_WB; i++ )
            {
                st_fx->prev_lpc_wb_fx[i] = lpc_wb[i + 1];
                move16();
            }
            FOR( i = 1; i < LPC_SHB_ORDER_LBR_WB + 1; i++ )
            {
                lpc_wb[i] = negate( lpc_wb[i] );
                move16();
            }
            lpc_wb[0] = 4096;
            move16();
        }
        ELSE
        {
            /* convert LSPs back into LP coeffs */
            lsp2lpc_fx( lpc_wb + 1, lsf_wb, st_fx->prev_lpc_wb_fx, LPC_SHB_ORDER_WB );
            FOR( i = 0; i < LPC_SHB_ORDER_WB; i++ )
            {
                st_fx->prev_lpc_wb_fx[i] = lpc_wb[i + 1];
                move16();
            }
            FOR( i = 1; i < LPC_SHB_ORDER_WB + 1; i++ )
            {
                lpc_wb[i] = negate( lpc_wb[i] );
                move16();
            }
            lpc_wb[0] = 4096;
            move16();
        }

        Copy( voice_factors, vf_modified, NB_SUBFR16k );
        IF( coder_type == VOICED )
        {
            FOR( i = 1; i < NB_SUBFR; i++ )
            {
                vf_modified[i] = add( mult_r( 26214, voice_factors[i] ), mult_r( 6554, voice_factors[i - 1] ) ); /* Q15 */  move16();
            }
            IF( st_fx->L_frame_fx != L_FRAME )
            {
                vf_modified[4] = add( mult_r( 26214, voice_factors[4] ), mult_r( 6554, voice_factors[3] ) ); /* Q15 */  move16();
            }
        }

        /* From low band excitation, generate highband excitation */
        Lmax = 0;
        FOR( cnt = 0;  cnt < L_FRAME32k; cnt++ )
        {
            Lmax = L_max( Lmax, L_abs( bwe_exc_extended[cnt] ) );
        }
        Q_bwe_exc = ( Lmax == 0 )?31:norm_l( Lmax );
        Q_bwe_exc = sub( Q_bwe_exc, 1 );
        Q_bwe_exc = add( Q_bwe_exc, add( Q_exc, Q_exc ) );

        find_max_mem_wb( st_fx, &n_mem );

        if( sub( Q_bwe_exc, st_fx->prev_Q_bwe_exc ) > n_mem )
        {
            Q_bwe_exc = add( st_fx->prev_Q_bwe_exc, n_mem );
        }

        test();
        if( uv_flag && sub( Q_bwe_exc, 20 ) > 0)
        {
            Q_bwe_exc = 20;
            move16(); /* restrict this to 21 due to the Q factor requireemnt of the random number generator (keep 1 bit headroom) */
        }

        prev_pow = 0;
        FOR( i = 0; i < L_SHB_LAHEAD / 4; i++ )
        {
            prev_pow = L_mac0( prev_pow, st_fx->state_syn_shbexc_fx[i], st_fx->state_syn_shbexc_fx[i] ); /*Q(2*(st_fx->prev_Q_bwe_exc-16))*/
        }

        rescale_genWB_mem( st_fx, sub( Q_bwe_exc, st_fx->prev_Q_bwe_exc ) );

        Copy( st_fx->old_bwe_exc_extended_fx, bwe_exc_extended_16, NL_BUFF_OFFSET );
        sc = sub( Q_bwe_exc, add( Q_exc, Q_exc ) );
        FOR( cnt = 0; cnt < L_FRAME32k; cnt++ )
        {
            bwe_exc_extended_16[cnt + NL_BUFF_OFFSET] = round_fx( L_shl( bwe_exc_extended[cnt], sc ) );
        }
        Copy( bwe_exc_extended_16 + L_FRAME32k, st_fx->old_bwe_exc_extended_fx, NL_BUFF_OFFSET );

        Copy( st_fx->state_syn_shbexc_fx, shaped_wb_excitation, L_SHB_LAHEAD / 4 );

        Q_bwe_exc_ext = sub( Q_bwe_exc, 16 );

        GenShapedWBExcitation_fx( shaped_wb_excitation + L_SHB_LAHEAD / 4, lpc_wb, exc4kWhtnd, st_fx->mem_csfilt_fx,
                                  st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->mem_genSHBexc_filt_down_wb2_fx,
                                  st_fx->mem_genSHBexc_filt_down_wb3_fx, st_fx->state_lpc_syn_fx, coder_type,
                                  bwe_exc_extended_16, Q_bwe_exc_ext, st_fx->bwe_seed_fx, vf_modified, uv_flag
                                  , st_fx->igf
                                );

        curr_pow = 0;
        FOR( i = 0; i < L_SHB_LAHEAD / 4; i++ )
        {
            curr_pow = L_mac0( curr_pow, shaped_wb_excitation[i + L_SHB_LAHEAD / 4], shaped_wb_excitation[i +
                               L_SHB_LAHEAD / 4] ); /* Q(2*Q_bwe_exc_ext) */
        }

        if( sub( voice_factors[0], 24576 ) > 0 )
        {
            curr_pow = L_shr( curr_pow, 2 ); /* Q(2*Q_bwe_exc_ext) */
        }

        Lscale = root_a_over_b_fx( curr_pow, shl_r( Q_bwe_exc_ext, 1 ), prev_pow,
                                   shl_r( sub( st_fx->prev_Q_bwe_exc, 16 ), 1 ), &exp );

        FOR( i = 0; i < L_SHB_LAHEAD / 4 - 1; i++ )
        {
            L_tmp = Mult_32_16( Lscale, shaped_wb_excitation[i] ); /* Q(16-exp+Q_bwe_exc_ext) */
            shaped_wb_excitation[i] = round_fx( L_shl( L_tmp, exp ) ); /* Q_bwe_exc_ext */
        }
        Lscale = root_a_fx( Lscale, 31 - exp, &exp );
        L_tmp = Mult_32_16( Lscale, shaped_wb_excitation[L_SHB_LAHEAD / 4 - 1] ); /* Q(16-exp+Q_bwe_exc_ext) */
        shaped_wb_excitation[L_SHB_LAHEAD / 4 - 1] = round_fx( L_shl( L_tmp, exp ) ); /* Q_bwe_exc_ext */

        /* Update SHB excitation */
        Copy( shaped_wb_excitation + L_FRAME16k / 4, st_fx->state_syn_shbexc_fx, L_SHB_LAHEAD / 4 );


        /* Adjust the subframe and frame gain of the synthesized shb signal */
        /* Scale the shaped excitation */
        ScaleShapedWB_fx( SHB_OVERLAP_LEN / 2, shaped_wb_excitation, st_fx->syn_overlap_fx, GainShape, GainFrame,
                          window_wb_fx, subwin_wb_fx,
                          Q_bwe_exc_ext
                          , st_fx->L_frame_fx
                          , 0
                          , &dummy
                          , dummy
                          , dummy2
                        );

        max = 0;
        FOR( i = 0; i < L_FRAME16k / 4; i++ )
        {
            max = s_max( max, shaped_wb_excitation[i] ); /*Q0*/
        }

        IF( max == 0 )
        {
            curr_frame_pow = 1;
            move16();
            n = 0;
            move16();
        }
        ELSE
        {
            n = norm_s( max );
            FOR( i = 0; i < L_FRAME16k / 4; i++ )
            {
                shaped_wb_excitation_frac[i] = shl( shaped_wb_excitation[i], n ); /*Q14*/ move16();
            }
            n = sub( 14, n );
            curr_frame_pow = 1;
            FOR( i = 0;  i < L_FRAME16k / 4; i++ )
            {
                L_tmp = L_mult( shaped_wb_excitation_frac[i], shaped_wb_excitation_frac[i] ); /*Q29*/
                curr_frame_pow = L_add( curr_frame_pow, L_shr( L_tmp, 7 ) ); /*Q22*/
            }
        }
        curr_frame_pow_exp = add( n, n );

        IF ( sub(st_fx->prev_frame_pow_exp, curr_frame_pow_exp) > 0 )
        {
            curr_frame_pow = L_shr( curr_frame_pow, sub( st_fx->prev_frame_pow_exp, curr_frame_pow_exp ) );
            curr_frame_pow_exp = st_fx->prev_frame_pow_exp;
        }
        ELSE
        {
            st_fx->prev_wb_bwe_frame_pow_fx = L_shr( st_fx->prev_wb_bwe_frame_pow_fx, sub( curr_frame_pow_exp, st_fx->prev_frame_pow_exp ) );
        }

        test();
        IF( !st_fx->bfi_fx && st_fx->prev_bfi_fx )
        {
            IF( L_sub( L_shr( curr_frame_pow, 1 ), st_fx->prev_wb_bwe_frame_pow_fx ) > 0 )
            {
                L_tmp = root_a_over_b_fx( st_fx->prev_wb_bwe_frame_pow_fx, 22, curr_frame_pow, 22, &exp );
                scale = round_fx( L_shl( L_tmp, exp ) ); /*Q15*/

                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                temp = round_fx( L_shl( L_tmp, exp ) ); /* Q15 */
            }
            ELSE
            {
                scale = temp = 32767;
                move16();/* Q15 */
            }

            FOR( j = 0; j < 8; j++ )
            {
                GainShape[2 * j] = mult_r( GainShape[2 * j], scale );
                GainShape[2 * j + 1] = mult_r( GainShape[2 * j + 1], scale );
                FOR( i = 0; i < L_FRAME16k / ( 4 * 8 ); i++ )
                {
                    shaped_wb_excitation[i + j * L_FRAME16k / ( 4 * 8 )] = mult_r( shaped_wb_excitation[i + j * L_FRAME16k / ( 4 * 8 )], scale );
                }
                IF( temp > 0 )
                {
                    IF( sub( scale, temp ) < 0 )
                    {
                        scale = div_s( scale, temp );
                    }
                    ELSE
                    {
                        scale = 32767;
                        move16();
                    }
                }
                ELSE
                {
                    scale = 0;
                    move16();
                }
            }
        }

        st_fx->prev_wb_bwe_frame_pow_fx = curr_frame_pow;
        move32();
        st_fx->prev_frame_pow_exp = curr_frame_pow_exp;
        move16();

        /* generate 16kHz SHB signal (6 - 8 kHz) from 2kHz signal */
        max = 0;
        move16();
        FOR( cnt = 0; cnt < ( L_FRAME16k + L_SHB_LAHEAD ) / 4; cnt++ )
        {
            if( abs_s( shaped_wb_excitation[cnt] ) > max )
            {
                max = abs_s( shaped_wb_excitation[cnt] );
            }
        }
        Qx = norm_s( max );
        Qx = ( max == 0 )?15:Qx;

        if ( max == 0 )
        {
            Qx = 15;
            move16();
        }

        Qx = sub( Qx, 1 ); /* 1 bit space for saturation */

        max = 0;
        move16();
        FOR( i = 0; i < 7; i++ )
        {
            if( abs_s( st_fx->state_lsyn_filt_shb_fx[i] ) > max )
                max = abs_s( st_fx->state_lsyn_filt_shb_fx[i] );
        }

        FOR( i = 0; i < 7; i++ )
        {
            if( abs_s( st_fx->state_lsyn_filt_dwn_shb_fx[i] ) > max )
                max = abs_s( st_fx->state_lsyn_filt_dwn_shb_fx[i] );
        }
        IF( L_sub(st_fx->output_Fs_fx, 32000) == 0 )
        {
            FOR ( i = 0; i < 2*ALLPASSSECTIONS_STEEP; i++ )
            {
                max = s_max(max, abs_s( st_fx->state_32and48k_WB_upsample_fx[i] ));
            }
        }
        IF( L_sub(st_fx->output_Fs_fx, 48000) == 0 )
        {
            FOR ( i = 0; i < INTERP_3_1_MEM_LEN; i++ )
            {
                max = s_max(max, abs_s( st_fx->mem_resamp_HB_fx[i] ));
            }
        }
        n_mem = max == 0?15:norm_s( max );
        n_mem = s_max( n_mem, 0 );

        if( sub( Qx, st_fx->prev_Qx ) > n_mem )
            Qx = add( st_fx->prev_Qx, n_mem );

        FOR( i = 0; i < ( L_FRAME16k + L_SHB_LAHEAD ) / 4; i++ )
        {
            shaped_wb_excitation[i] = shl( shaped_wb_excitation[i], Qx );
            move16();
        }

        FOR( i = 0; i < 7; i++ )
        {
            st_fx->state_lsyn_filt_shb_fx[i] = shl( st_fx->state_lsyn_filt_shb_fx[i], sub( Qx, st_fx->prev_Qx ) );
            move16();
        }

        FOR( i = 0; i < 7; i++ )
        {
            st_fx->state_lsyn_filt_dwn_shb_fx[i] = shl( st_fx->state_lsyn_filt_dwn_shb_fx[i], sub( Qx, st_fx->prev_Qx ) );
            move16();
        }

        GenWBSynth_fx( shaped_wb_excitation, error, st_fx->state_lsyn_filt_shb_fx, st_fx->state_lsyn_filt_dwn_shb_fx );

        FOR( i = 0; i < L_FRAME16k; i++ )
        {
            synth[i] = mult_r( error[i], 21299 );
            move16();
        }

        max = 0;
        move16();
        FOR( cnt = 0; cnt < L_FRAME16k; cnt++ )
        {
            max = s_max( max, abs_s( synth[cnt] ) );
        }

        IF( max == 0 )
        {
            st_fx->last_wb_bwe_ener_fx = 0;
            move16();
        }
        ELSE
        {
            n = norm_s( max );
            FOR( cnt = 0; cnt < L_FRAME16k; cnt++ )
            {
                synth_frac[cnt] = shl( synth[cnt], n ); /*Q14*/ move16();
            }
            n = sub( sub( 14, n ), Qx );

            Lacc = 0;
            FOR( i = 0; i < L_FRAME16k; i++ )
            {
                L_tmp = L_mult( synth_frac[i], synth_frac[i] ); /* Q29 */
                Lacc = L_add( Lacc, L_shr( L_tmp, 7 ) ); /* Q22 */
            }

            L_tmp = Mult_32_16( Lacc, 102 ); /* Q22 */
            exp = norm_l( L_tmp );
            tmp = round_fx( L_shl( L_tmp, exp ) );
            exp = sub( add( exp, 22 ), 30 );
            tmp = div_s( 16384, tmp );
            L_tmp = Isqrt_lc( L_deposit_h( tmp ), &exp ); /* Q(31-exp) */
            st_fx->last_wb_bwe_ener_fx = round_fx( L_shl( L_tmp, add( exp, n - 12 ) ) ); /*  Q3 */
        }


        IF( L_sub(st_fx->output_Fs_fx, 32000) == 0 )  /* 32kHz sampling rate, but only WB output - interpolate */
        {
            Scale_sig(st_fx->state_32and48k_WB_upsample_fx, 2*ALLPASSSECTIONS_STEEP, sub( Qx, st_fx->prev_Qx ));
            Interpolate_allpass_steep_fx( synth, st_fx->state_32and48k_WB_upsample_fx, L_FRAME16k, upsampled_synth );
            Copy( upsampled_synth, synth, L_FRAME32k );
        }
        ELSE IF( L_sub(st_fx->output_Fs_fx, 48000) == 0 )
        {
            Scale_sig(st_fx->mem_resamp_HB_fx, INTERP_3_1_MEM_LEN, sub( Qx, st_fx->prev_Qx ));
            interpolate_3_over_1_allpass_fx( synth, L_FRAME16k, upsampled_synth, st_fx->mem_resamp_HB_fx, allpass_poles_3_ov_2 );
            Copy( upsampled_synth, synth, L_FRAME48k );
        }
        ELSE IF( L_sub(st_fx->output_Fs_fx, 8000) == 0 )
        {
            /* set high band synthesis to zero when NB is output; This will allow the wb synthesis to be carried out and
                subsequently populate all memories for use when the codec switches back to output_fs=16kHz */
            set16_fx( synth, 0, L_FRAME16k );
            Qx = 15;
        }
    }
    ELSE
    {
        f = 5461;
        move16();/* Q15 */
        inc = 5461;
        move16();/* Q15 */
        FOR( i = 0; i < LPC_SHB_ORDER_WB; i++ )
        {
            lsf_wb[i] = f;
            move16();/*Q15*/
            f = add( f, inc );
            move16();
        }
        GainFrame = 0; /* Q18 */
        Qx = 0;
        Q_bwe_exc = 31;
        st_fx->prev_wb_bwe_frame_pow_fx = FL2WORD32_SCALE(0.001f, 31-22); /* Q22 */
        st_fx->prev_frame_pow_exp = 0;
        move16();
    }

    /* Update previous frame parameters for FEC */
    Copy( lsf_wb, st_fx->lsp_prevfrm_fx, LPC_SHB_ORDER_WB );
    st_fx->GainFrame_prevfrm_fx = GainFrame; /* Q18 */

    if( !st_fx->bfi_fx )
    {
        st_fx->GainAttn_fx = 32767;
        move16();
    }

    *Q_synth = Qx;
    move16();

    st_fx->prev_Q_bwe_exc = Q_bwe_exc;
    move16();
    st_fx->prev_Qx = Qx;
    move16();

    return;
}



/*======================================================================================*/
/* FUNCTION      : void swb_tbe_dec_fx ()                                               */
/*--------------------------------------------------------------------------------------*/
/* PURPOSE       : SWB TBE decoder, 6 - 14 kHz (or 7.5 - 15.5 kHz) band decoding module */
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                                   */
/*  _(Word16) coder_type         : coding type                                          */
/*  _(Word16*) bwe_exc_extended  :bandwidth extended exciatation  Q0                    */
/*  _(Word16[]) voice_factors    :voicing factors          Q15                          */
/*  _(Word16*) Q_white_exc       :Q Format of White Exc                                 */
/*  _(Word16*) Q_synth           :Q Format of Synthesis                                 */
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                                   */
/*   _(Word16*)synth            : SHB synthesis/final synthesis          Q_white_exc    */
/*   _(Word16*)White_exc16k     : shaped white excitation for the FB TBE  Q_synth       */
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                                             */
/*   _ Decoder_State_fx *st_fx:      : Decoder state structure                          */
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                                   */
/*           _ None                                                                     */
/*--------------------------------------------------------------------------------------*/
/* CALLED FROM : RX                                                                     */
/*======================================================================================*/
void swb_tbe_dec_fx(
    Decoder_State_fx* st_fx,    /* i/o: decoder state structure                 */
    const   Word16 coder_type,          /* i  : coding type                             */
    Word32* bwe_exc_extended,           /* i  : bandwidth extended excitation  2*Q_exc  */
    Word16  Q_exc,
    const   Word16 voice_factors[],     /* i  : voicing factors                         */
    const   Word16 old_syn_12k8_16k[],  /* i  : low band synthesis                      */
    Word16* White_exc16k,               /* o  : shaped white excitation for the FB TBE  */
    Word16* Q_white_exc,
    Word16* synth,                      /* o  : SHB synthesis/final synthesis           */
    Word16* Q_synth,
    Word16* pitch_buf                   /* i  : pitch buffer Q6                         */
)
{
    Word16 i;
    Word16 shaped_shb_excitation [ L_FRAME16k + L_SHB_LAHEAD ];
    Word16 bwe_exc_extended_16[L_FRAME32k+NL_BUFF_OFFSET];
    Word16 lsf_shb[LPC_SHB_ORDER], lpc_shb[LPC_SHB_ORDER + 1], GainShape[NUM_SHB_SUBFR];
    Word32 GainFrame;
    Word16 error[L_FRAME32k];
    Word32 L_ener;
    Word16 ener;
    Word16 is_fractive;
    Word32 prev_pow, curr_pow, Lscale;
    Word16 scale;
    Word16 exp, tmp;
    Word16 j, cnt ;
    Word16 n_mem, n_mem2, Qx, sc;

    Word16 n_mem3;

    Word32 Lmax, L_tmp;
    Word16 frac;

    Word32 L_tmp1, L_tmp2;
    Word16 expa, expb;
    Word16 fraca, fracb;
    Word16 GainShape_tmp[NUM_SHB_SUBGAINS];
    Word16 Q_bwe_exc;
    Word16 Q_shb;
    Word16 vf_modified[NB_SUBFR16k];
    Word16 stemp;

    Word16 tilt_swb_fec;
    Word16 Q_bwe_exc_fb;


    Word16 lsp_shb_1[LPC_SHB_ORDER], lsp_shb_2[LPC_SHB_ORDER], lsp_temp[LPC_SHB_ORDER];
    Word16 lpc_shb_sf[4*(LPC_SHB_ORDER+1)];
    const Word16 *ptr_lsp_interp_coef;
    Word32 shb_ener_sf_32;
    Word16 shb_res_gshape[NB_SUBFR16k];
    Word16 mixFactors;
    Word16 vind;
    Word16 shb_res_dummy[L_FRAME16k];
    Word16 shaped_shb_excitationTemp[L_FRAME16k];
    Word32 ener_tmp[NUM_SHB_SUBGAINS];
    Word16 pitch_fx;
    Word16 l_subframe_fx;
    Word16 formant_fac;
    Word16 lsf_diff[LPC_SHB_ORDER], w[LPC_SHB_ORDER];
    Word16 refl[M];
    Word16 tilt_para;
    Word16 tmp1,tmp2;
    Word16 f_fx, inc_fx;
    Word32 GainFrame_prevfrm_fx;

    Word16 synth_scale_fx;
    Word16 mean_vf;
    Word16 exp_ener, inv_ener;
    Word32 prev_ener_ratio_fx=0; /* initialize just to avoid compiler warning */
    Word16 max,n,temp,shaped_shb_excitation_frac [ L_FRAME16k + L_SHB_LAHEAD ];
    Word32 curr_frame_pow;
    Word16 curr_frame_pow_exp;
    Word32 L_prev_ener_shb;
    /* initializations */
    GainFrame      = L_deposit_l(0);
    mixFactors     = 0;
    move16();
    shb_ener_sf_32 = L_deposit_l(0);
    set16_fx( shaped_shb_excitationTemp, 0, L_FRAME16k );
    st_fx->shb_dtx_count_fx = 0;
    move16();
    is_fractive = 0;
    move16();
    set16_fx( shb_res_gshape, FL2WORD16_SCALE(0.1f, 15-14), NB_SUBFR16k );  /* Q14 */
    Q_shb = 0;   /* high band target Q factor set to zero */

    L_tmp = calc_tilt_bwe_fx(old_syn_12k8_16k, st_fx->Q_syn2, st_fx->L_frame_fx);
    tilt_swb_fec = round_fx(L_shl(L_tmp, 3));
    /* i: old_syn_12k8_16k in st_fx->Q_syn2 */
    /* o: tilt_swb_fec in Q11 */


    /* WB/SWB bandwidth switching */
    test();
    test();
    IF( ( sub(st_fx->tilt_wb_fx, 10240) > 0 && sub(st_fx->clas_dec, UNVOICED_CLAS) == 0 ) || sub(st_fx->tilt_wb_fx, 20480) > 0 )
    {
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF( (st_fx->prev_fractive_fx == 0 &&
             (L_sub( st_fx->prev_enerLH_fx, L_shl( st_fx->enerLH_fx, 1 ) ) < 0 && L_sub( st_fx->prev_enerLH_fx, L_shr( st_fx->enerLH_fx, 1 ) ) > 0
              && L_sub( st_fx->prev_enerLL_fx, L_shl( st_fx->enerLL_fx, 1 ) ) < 0 && L_sub( st_fx->prev_enerLL_fx, L_shr( st_fx->enerLL_fx, 1 ) ) > 0))
            || (sub(st_fx->prev_fractive_fx,1) == 0 &&
                L_sub(L_shr(st_fx->prev_enerLH_fx,2), Mult_32_16(st_fx->enerLH_fx,24576)) > 0 )  /* 24576 in Q13*/
            || (L_sub(L_shr(st_fx->enerLL_fx,1), Mult_32_16(st_fx->enerLH_fx, 24576)) >  0  &&  /*24576 = 1.5 in Q14*/
                sub(st_fx->tilt_wb_fx, 20480) < 0  )/* 20480 = 10 in Q11*/
          )
        {
            is_fractive = 0;
            move16();
        }
        ELSE
        {
            is_fractive = 1;
            move16();
        }
    }

    /* WB/SWB bandwidth switching */
    IF( st_fx->bws_cnt_fx > 0 )
    {
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF( (sub(st_fx->last_extl_fx, SWB_TBE) != 0 && sub(st_fx->last_extl_fx, FB_TBE) != 0 &&
             !(L_sub(L_shr(st_fx->prev_enerLH_fx, 1), st_fx->enerLH_fx) < 0 &&L_sub(st_fx->prev_enerLH_fx, L_shr(st_fx->enerLH_fx,1)>0)))
            || sub(st_fx->last_core_fx, ACELP_CORE) != 0
            || (sub(st_fx->last_core_fx, ACELP_CORE) == 0 && L_sub(L_abs(L_sub(st_fx->last_core_brate_fx, st_fx->core_brate_fx)), 3600) > 0)
            ||  sub((is_fractive ^ st_fx->prev_fractive_fx), 1) == 0 )
        {
            f_fx = 1489;    /*Q15*/
            inc_fx = 1489;  /*Q15*/
            IF(sub(is_fractive, 1) == 0)
            {
                Copy(lsf_tab_fx, st_fx->lsp_prevfrm_fx, LPC_SHB_ORDER);
            }
            ELSE
            {
                FOR (i=0; i<LPC_SHB_ORDER; i++)
                {
                    st_fx->lsp_prevfrm_fx[i] = f_fx;
                    move16();
                    f_fx = add(f_fx, inc_fx);
                }
            }
            set16_fx( GainShape, 11587, NUM_SHB_SUBFR );
        }
        ELSE
        {
            if(sub(st_fx->prev_GainShape_fx, 11587) > 0)
            {
                st_fx->prev_GainShape_fx = 11587;
                move16();
            }
            set16_fx( GainShape, st_fx->prev_GainShape_fx, NUM_SHB_SUBFR );
        }

        Copy( st_fx->lsp_prevfrm_fx, lsf_shb, LPC_SHB_ORDER );
        set16_fx( shb_res_gshape, FL2WORD16_SCALE(0.2f, 15-14), NB_SUBFR16k );  /* Q14 */
    }
    ELSE /* No bandwidth switching */
    {

        IF( !st_fx->bfi_fx )
        {
            IF(st_fx->use_partial_copy)
            {
                Copy( st_fx->lsp_prevfrm_fx, lsf_shb, LPC_SHB_ORDER );
                set16_fx( GainShape, RECIP_ROOT_EIGHT_FX, NUM_SHB_SUBFR );

                IF( sub(st_fx->rf_frame_type,RF_NELP) == 0 )
                {
                    /* Frame gain */
                    GainFrame = L_mac( SHB_GAIN_QLOW_FX, st_fx->rf_indx_tbeGainFr[0], SHB_GAIN_QDELTA_FX );
                    move32();/*Q18*/
                    L_tmp = Mult_32_16( GainFrame, 27213 ); /*Q16*/   /* 3.321928 in Q13 */

                    frac = L_Extract_lc( L_tmp, &exp );
                    L_tmp = Pow2( 30, frac );
                    GainFrame = L_shl( L_tmp, sub( exp, 12 ) ); /*Q18*/

                    IF( sub(st_fx->core_fx,ACELP_CORE) == 0 && sub(st_fx->last_core_fx,ACELP_CORE) == 0
                    && !st_fx->prev_use_partial_copy && sub(st_fx->prev_coder_type_fx,UNVOICED) == 0
                    && L_sub(GainFrame,st_fx->GainFrame_prevfrm_fx) != 0 && sub(st_fx->next_coder_type,GENERIC) != 0 )
                    {
                        /*GainFrame = 0.2f*GainFrame + 0.8f*st_fx->GainFrame_prevfrm_fx;*/
                        GainFrame = L_add(Mult_32_16(st_fx->GainFrame_prevfrm_fx, 26214), Mult_32_16(GainFrame, 6553));
                    }
                }
                ELSE
                {
                    temp = 0;
                    move16();
                    /* Frame gain */
                    SWITCH (st_fx->rf_indx_tbeGainFr[0])
                    {
                    case 0:
                        GainFrame = 131072; /* 0.5f in Q18 */
                        IF(L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(1.25, 31-18)) <= 0) temp = FL2WORD16(0.8);
                        move16();
                        BREAK;
                    case 1:
                        GainFrame = 524288; /* 2.0f in Q18 */
                        IF(L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(1.25, 31-18)) > 0 && L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(3, 31-18)) <= 0) temp = FL2WORD16(0.8);
                        move16();
                        test();
                        BREAK;
                    case 2:
                        GainFrame = 1048576;/* 4.0f in Q18 */
                        IF(L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(3, 31-18)) > 0 && L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(6, 31-18)) <= 0) temp = FL2WORD16(0.8);
                        move16();
                        test();
                        BREAK;
                    case 3:
                        GainFrame = 2097152;/* 8.0f in Q18 */
                        IF(L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(6, 31-18)) > 0 && L_sub(st_fx->GainFrame_prevfrm_fx, FL2WORD32_SCALE(16, 31-18)) <= 0) temp = FL2WORD16(0.8);
                        move16();
                        test();
                        BREAK;
                    default:
                        fprintf(stderr, "RF SWB-TBE gain bits not supported.");
                    }
                    GainFrame = L_add(Mult_32_16(st_fx->GainFrame_prevfrm_fx, temp), Mult_32_16(GainFrame, sub(32767,temp)));
                    IF(sub(st_fx->core_fx,ACELP_CORE) == 0 && sub(st_fx->last_core_fx,ACELP_CORE) == 0)
                    {
                        IF(!st_fx->prev_use_partial_copy && sub(st_fx->last_coder_type_fx, VOICED) == 0 && sub(st_fx->rf_frame_type,RF_GENPRED) == 0 && L_sub(GainFrame, 2097152) > 0 && L_sub(GainFrame, 3059606) < 0 )
                        {
                            GainFrame = Mult_32_16(GainFrame,9830);
                        }
                    }
                }
            }
            ELSE
            {

                /* de-quantization */
                dequantizeSHBparams_fx_9_1( st_fx, st_fx->extl_fx, st_fx->extl_brate_fx, lsf_shb, GainShape, &GainFrame, &stemp,
                &shb_ener_sf_32, shb_res_gshape, &mixFactors );
                Q_shb = 0;
                move16();
                /* o: shb_ener_sf_32 in (2*Q_shb) */
                /* o: shb_res_gshape in Q14 */
                /* o: GainShape Q15 */
                /* o: GainFrame Q18 */
            }
        }
        ELSE /* FER concealment of TBE parameters */
        {
            Copy( st_fx->lsp_prevfrm_fx, lsf_shb, LPC_SHB_ORDER );

            /* Gain shape concealment */
            IF( sub(st_fx->codec_mode, MODE1) == 0 )
            {
                /* Gradient based GS estimation */
                gradientGainShape(st_fx, GainShape, &GainFrame);
                /* o: GainShape[16] in Q15 */
                /* o: GainFrame in Q18 */
            }
            ELSE
            {
                FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
                {
                    FOR( j = 0; j < 4; j++ )
                    {
                        GainShape[i * 4 + j] = mult_r( st_fx->cummulative_damping, st_fx->GainShape_Delay[4+i]);
                        move16();
                    }
                }
                IF( sub( tilt_swb_fec, 8<<11 ) > 0 )   /* tilt_swb_fec in Q11 */
                {
                    IF ( sub(st_fx->nbLostCmpt, 1) == 0 )
                    {
                        GainFrame = Mult_32_16(st_fx->GainFrame_prevfrm_fx, FL2WORD16(0.6f));
                    }
                    ELSE IF( sub(st_fx->nbLostCmpt, 2) == 0 )
                    {
                        GainFrame = Mult_32_16(st_fx->GainFrame_prevfrm_fx, FL2WORD16(0.35f));
                    }
                    ELSE
                    {
                        GainFrame = Mult_32_16(st_fx->GainFrame_prevfrm_fx, FL2WORD16(0.2f));
                    }
                    GainFrame = Mult_32_16( GainFrame, st_fx->cummulative_damping);
                }
                ELSE
                {
                    GainFrame = st_fx->GainFrame_prevfrm_fx;
                    move16(); /* gain locking */
                }
            }

            /* FER concealment for 24.4kbps and 32kbps */
            test();
            IF(L_sub(st_fx->total_brate_fx,ACELP_24k40) == 0 || L_sub(st_fx->total_brate_fx, ACELP_32k) == 0)
            {
                IF(sub(st_fx->codec_mode, MODE1) == 0)
                {
                    /*scale = st->prev1_shb_ener_sf/root_a(st->prev2_shb_ener_sf * st->prev3_shb_ener_sf); */
                    L_tmp = Mult_32_32(st_fx->prev2_shb_ener_sf_fx, st_fx->prev3_shb_ener_sf_fx); /* 2*18 + 1 - 32 => Q5*/
                    L_tmp = root_a_fx(L_tmp, 5, &exp); /* Q = 31-exp */
                    tmp = round_fx( L_shl(L_tmp, exp) ); /* Q15 */
                    tmp1 = round_fx( L_shr(st_fx->prev1_shb_ener_sf_fx, 1) );  /* Q14, prepare for division */

                    test();
                    IF(sub(tmp1, tmp) > 0 || tmp == 0)
                    {
                        scale = 32767;
                        move16();
                    }
                    ELSE
                    {
                        scale = div_s(tmp1, tmp);   /* Q15 */
                    }
                    /*scale = st->prev_res_shb_gshape * min(scale, 1.0f); */
                    tmp = s_min(scale, 32767);
                    scale = mult_r(st_fx->prev_res_shb_gshape_fx, tmp);

                    test();
                    IF( L_sub( L_shr(st_fx->prev2_shb_ener_sf_fx, 1), st_fx->prev1_shb_ener_sf_fx ) > 0  ||
                        L_sub( L_shr(st_fx->prev3_shb_ener_sf_fx, 1), st_fx->prev2_shb_ener_sf_fx ) > 0  )
                    {
                        /* shb_ener_sf_32 = 0.5f * scale * st_fx->prev1_shb_ener_sf_fx; */
                        shb_ener_sf_32 = L_shr( Mult_32_16( st_fx->prev1_shb_ener_sf_fx, scale ), 1);

                        if( sub(st_fx->nbLostCmpt, 1) > 0)
                        {
                            /* shb_ener_sf_32 *= 0.5f; */
                            shb_ener_sf_32 = L_shr(shb_ener_sf_32, 1);
                        }
                    }
                    ELSE
                    {
                        /* shb_ener_sf = scale * scale * st_fx->prev1_shb_ener_sf_fx; */
                        L_tmp = L_mult(scale, scale);  /* Q31 */
                        shb_ener_sf_32 = Mult_32_16(st_fx->prev1_shb_ener_sf_fx, round_fx(L_tmp));

                        /* GainFrame *= scale; */
                        GainFrame = Mult_32_16(GainFrame, scale);
                    }
                }
                ELSE
                {
                    test();
                    IF( L_sub( L_shr(st_fx->prev2_shb_ener_sf_fx, 1), st_fx->prev1_shb_ener_sf_fx ) > 0 ||
                    L_sub( L_shr(st_fx->prev3_shb_ener_sf_fx, 1), st_fx->prev2_shb_ener_sf_fx ) > 0 )
                    {
                        /* shb_ener_sf_32 = 0.5f * st->cummulative_damping * st_fx->prev1_shb_ener_sf_fx; */
                        shb_ener_sf_32 = L_shr( Mult_32_16( st_fx->prev1_shb_ener_sf_fx, st_fx->cummulative_damping ), 1 );
                    }
                    ELSE
                    {
                        shb_ener_sf_32 = Mult_32_16( st_fx->prev1_shb_ener_sf_fx, st_fx->cummulative_damping );
                    }
                }
            }

            shb_ener_sf_32 = L_max( shb_ener_sf_32, FL2WORD32_SCALE(1.0f, 31-0) );
            mixFactors = st_fx->prev_mixFactors_fx;

            IF(sub(st_fx->codec_mode, MODE1) == 0)
            {
                set16_fx( shb_res_gshape, FL2WORD16_SCALE(0.2f, 15-14), 5 );  /* Q14 */
            }
            ELSE
            {
                set16_fx( shb_res_gshape, FL2WORD16_SCALE(1.0f, 15-14), 5 );  /* Q14 */
            }
        }
    }

    /* get the gainshape delay */
    Copy( &st_fx->GainShape_Delay[4], &st_fx->GainShape_Delay[0], NUM_SHB_SUBFR / 4 );
    IF ( (st_fx->rf_flag != 0) || L_sub(st_fx->total_brate_fx, ACELP_9k60) == 0 )
    {
        FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
        {
            st_fx->GainShape_Delay[i + 4] = s_min( s_max( GainShape[i * 4], FL2WORD16(0.1f) ), FL2WORD16(0.5f) );
            move16();
        }
    }
    ELSE
    {
        FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
        {
            st_fx->GainShape_Delay[i + 4] = GainShape[i * 4];
            move16();
        }
    }

    /* voice factor modification to limit any spurious jumps in the middle of voiced subframes*/
    /* mean(voice_factors[i], 4); */
    L_tmp = L_mult(voice_factors[0], 8192);
    L_tmp = L_mac(L_tmp, voice_factors[1], 8192);
    L_tmp = L_mac(L_tmp, voice_factors[2], 8192);
    mean_vf = mac_r(L_tmp, voice_factors[3], 8192);

    Copy( voice_factors, vf_modified, NB_SUBFR16k );

    test();
    IF( sub(coder_type, VOICED) == 0 || sub(mean_vf, FL2WORD16(0.4f) ) > 0 )
    {
        FOR( i = 1; i < NB_SUBFR; i++ )
        {
            L_tmp = L_mult(voice_factors[i], FL2WORD16(0.8f));
            vf_modified[i] = mac_r(L_tmp, voice_factors[i-1], FL2WORD16(0.2f));
            move16();
        }
        IF( sub(st_fx->L_frame_fx, L_FRAME) != 0 )
        {
            L_tmp = L_mult(voice_factors[4], FL2WORD16(0.8f));
            vf_modified[4] = mac_r(L_tmp, voice_factors[3], FL2WORD16(0.2f));
            move16();
        }
    }

    /* convert quantized LSFs to LSPs for interpolation */
    E_LPC_lsf_lsp_conversion(lsf_shb, lsp_shb_2, LPC_SHB_ORDER);

    test();
    IF( sub(st_fx->last_extl_fx, SWB_TBE)  == 0 || sub(st_fx->last_extl_fx, FB_TBE) == 0)
    {
        /* SHB LSP values from prev. frame for interpolation */
        Copy(st_fx->swb_lsp_prev_interp_fx, lsp_shb_1, LPC_SHB_ORDER);
    }
    ELSE
    {
        /* Use current frame's LSPs; in effect no interpolation */
        Copy(lsp_shb_2, lsp_shb_1, LPC_SHB_ORDER);
    }

    IF( st_fx->bws_cnt_fx == 0 && st_fx->bws_cnt1_fx == 0 && st_fx->rf_flag_last == 0 && st_fx->use_partial_copy == 0 )
    {
        lsf_diff[0] = 16384;
        move16();   /*Q15*/
        lsf_diff[sub(LPC_SHB_ORDER,1)] = 16384;
        move16();   /*Q15*/
        FOR(i=1; i < LPC_SHB_ORDER-1; i++)
        {
            lsf_diff[i] = sub(lsf_shb[i],lsf_shb[sub(i,1)]);
            move16();
        }

        a2rc_fx (st_fx->cur_sub_Aq_fx+1, refl,  M);
        tmp = add(16384, shr(refl[0],1));      /*Q14*/
        tmp1 = mult(27425,tmp);
        tmp1 = mult(tmp1,tmp);                 /*Q10*/
        tmp2 = shr(mult(31715,tmp),2);         /*Q10*/
        tilt_para = add(sub(tmp1,tmp2),1335);  /*Q10*/

        test();
        IF(sub(st_fx->last_extl_fx,SWB_TBE) != 0 && sub(st_fx->last_extl_fx,FB_TBE) != 0)
        {
            FOR(i=0; i<LPC_SHB_ORDER; i++)
            {
                st_fx->prev_lsf_diff_fx[i] = shr(lsf_diff[i], 1);
                move16();
            }
        }

        IF( L_sub(st_fx->total_brate_fx,ACELP_16k40) <= 0 )
        {
            test();
            test();
            test();
            test();
            test();
            IF(!(sub(st_fx->prev_tilt_para_fx,5120) > 0 && (sub(coder_type,TRANSITION) == 0 || sub(tilt_para,1024) < 0)) &&
               !(((sub(st_fx->prev_tilt_para_fx,3072) < 0 && sub(st_fx->prev_coder_type_fx,VOICED) >= 0)) && sub(tilt_para,5120) > 0))
            {
                FOR( i = 1; i < LPC_SHB_ORDER-1; i++ )
                {
                    IF(sub(lsf_diff[i],st_fx->prev_lsf_diff_fx[i]) < 0)
                    {
                        tmp = mult(26214,lsf_diff[i]);
                        tmp = div_s(tmp,st_fx->prev_lsf_diff_fx[i]);
                        tmp = s_max(tmp,16384);
                        w[i] = s_min(tmp,32767);
                        move16();
                    }
                    ELSE
                    {
                        tmp = mult(26214,st_fx->prev_lsf_diff_fx[i]);
                        tmp = div_s(tmp,lsf_diff[i]);
                        tmp = s_max(tmp,16384);
                        w[i] = s_min(tmp,32767);
                        move16();
                    }
                }
                w[0] = w[1];
                w[sub(LPC_SHB_ORDER,1)] = w[sub(LPC_SHB_ORDER,2)];

                FOR( i = 0; i < LPC_SHB_ORDER; i++ )
                {
                    tmp1 = mult(lsp_shb_1[i],sub(32767,w[i]));
                    tmp2 = mult(lsp_shb_2[i],w[i]);
                    lsp_temp[i] =add(tmp1,tmp2);
                    move16();
                }
            }
            ELSE
            {
                Copy(lsp_shb_2, lsp_temp, LPC_SHB_ORDER);
            }
        }
        Copy(lsf_diff, st_fx->prev_lsf_diff_fx, LPC_SHB_ORDER);
        st_fx->prev_tilt_para_fx = tilt_para;
    }
    ELSE
    {
        Copy(lsp_shb_2, lsp_temp, LPC_SHB_ORDER);
    }

    test();
    IF ( L_sub(st_fx->total_brate_fx, ACELP_24k40 ) == 0 || L_sub( st_fx->total_brate_fx, ACELP_32k) == 0 )
    {
        /* ---------- SHB LSP interpolation ---------- */
        ptr_lsp_interp_coef = interpol_frac_shb; /*Q15*/
        FOR( j = 0; j < 4; j++ )
        {
            FOR( i = 0; i < LPC_SHB_ORDER; i++ )
            {
                /*lsp_temp_fx[i] =  lsp_shb_1_fx[i]*(*ptr_lsp_interp_coef_fx) */
                /*                + lsp_shb_2_fx[i]*(*(ptr_lsp_interp_coef_fx+1));*/
                L_tmp = L_mult(lsp_shb_1[i], (*ptr_lsp_interp_coef));
                lsp_temp[i] = mac_r(L_tmp, lsp_shb_2[i], (*(ptr_lsp_interp_coef+1)));
                move16();
            }
            ptr_lsp_interp_coef += 2;

            /* convert from lsp to lsf */
            /*old code: lsp2lsf_fx(lsp_temp, lsp_temp, LPC_SHB_ORDER, INT_FS_FX); */ /* input lsp_temp_fx in Q15*/

            tmp = i_mult(j, (LPC_SHB_ORDER+1));
            /* convert LSPs to LP coefficients */
            E_LPC_f_lsp_a_conversion(lsp_temp, lpc_shb_sf+tmp, LPC_SHB_ORDER);
            /* Bring the LPCs to Q12 */
            Copy_Scale_sig( lpc_shb_sf+tmp, lpc_shb_sf+tmp, LPC_SHB_ORDER+1, sub(norm_s(lpc_shb_sf[tmp]),2) );
        }
    }
    /*ELSE*/
    {
        /* for 13.2 and 16.4kbps */
        E_LPC_f_lsp_a_conversion(lsp_temp, lpc_shb, LPC_SHB_ORDER);
        Copy_Scale_sig( lpc_shb, lpc_shb, LPC_SHB_ORDER+1, sub(norm_s(lpc_shb[0]),2) );  /* Q12 */
    }

    /* Save the SWB LSP values from current frame for interpolation */
    Copy(lsp_shb_2, st_fx->swb_lsp_prev_interp_fx, LPC_SHB_ORDER);
    /* lsp_shb_2_fx in Q15 */

    /* save the shb_ener Q18, prev_resgainshape Q14,  and mixFactor Q15 values */
    st_fx->prev3_shb_ener_sf_fx = st_fx->prev2_shb_ener_sf_fx;
    st_fx->prev2_shb_ener_sf_fx = st_fx->prev1_shb_ener_sf_fx;
    st_fx->prev1_shb_ener_sf_fx = shb_ener_sf_32;
    st_fx->prev_res_shb_gshape_fx = shb_res_gshape[4];
    st_fx->prev_mixFactors_fx = mixFactors;

    /* SWB CNG/DTX - update memories */
    Copy( st_fx->lsp_shb_prev_fx, st_fx->lsp_shb_prev_prev_fx, LPC_SHB_ORDER ); /* Q15 */
    Copy( lsf_shb, st_fx->lsp_shb_prev_fx, LPC_SHB_ORDER );                     /* Q15 */

    /* vind = (short)(mixFactors*8.0f); */
    vind = shl(mixFactors,3-15); /* 3 for mpy by 8.0f, -15 to bring it to Q0 */
    /* i: mixFactors in Q15 */
    /* o: vind in Q0        */

    /* Determine formant PF strength */
    formant_fac = swb_formant_fac_fx( lpc_shb[1], &st_fx->tilt_mem_fx );
    /* o: formant_fac in Q15 */

    /* -------- start of  memory rescaling  -------- */
    /* ----- calculate optimum Q_bwe_exc and rescale memories accordingly ----- */
    Lmax = 0;
    FOR( cnt = 0; cnt < L_FRAME32k; cnt++ )
    {
        Lmax = L_max( Lmax, L_abs( bwe_exc_extended[cnt] ) );
    }
    Q_bwe_exc = norm_l( Lmax );
    if(Lmax == 0)
    {
        Q_bwe_exc = 31;
    }
    Q_bwe_exc = add( Q_bwe_exc, add( Q_exc, Q_exc ) );

    /* Account for any outliers in the memories from previous frame for rescaling to avoid saturation */
    find_max_mem_dec( st_fx, &n_mem, &n_mem2
                      , &n_mem3
                    );  /* for >=24.4, use n_mem2 lpc_syn, shb_20sample, and mem_stp_swb_fx memory */

    tmp = add( st_fx->prev_Q_bwe_exc, n_mem );
    if( sub( Q_bwe_exc, tmp) > 0 )
    {
        Q_bwe_exc = tmp;
    }

    /* rescale the memories if Q_bwe_exc is different from previous frame */
    sc = sub( Q_bwe_exc, st_fx->prev_Q_bwe_exc );
    IF( sc != 0 )
    {
        rescale_genSHB_mem_dec( st_fx, sc );
    }

    /* rescale the bwe_exc_extended and bring it to 16-bit single precision with dynamic norm  */
    Copy( st_fx->old_bwe_exc_extended_fx, bwe_exc_extended_16, NL_BUFF_OFFSET );
    sc = sub( Q_bwe_exc, add( Q_exc, Q_exc ) );

    FOR( cnt = 0; cnt < L_FRAME32k; cnt++ )
    {
        bwe_exc_extended_16[cnt + NL_BUFF_OFFSET] = round_fx( L_shl( bwe_exc_extended[cnt], sc ) );
    }
    Copy( bwe_exc_extended_16 + L_FRAME32k, st_fx->old_bwe_exc_extended_fx, NL_BUFF_OFFSET );

    /* state_syn_shbexc_fx is kept at (st_fx->prev_Q_bwe_syn) for 24.4/32kbps or is kept at Q_bwe_exc for 13.2/16.4kbps */
    Copy( st_fx->state_syn_shbexc_fx, shaped_shb_excitation, L_SHB_LAHEAD );

    /* save the previous Q factor (32-bit) of the buffer */
    st_fx->prev_Q_bwe_exc = Q_bwe_exc;
    move16();

    Q_bwe_exc = sub( Q_bwe_exc, 16 );   /* Q_bwe_exc reflecting the single precision dynamic norm-ed buffers from here */

    /* -------- end of rescaling memories -------- */

    /* Calculate the 6 to 14 kHz (or 7.5 - 15.5 kHz) SHB excitation signal from the low band ACELP core excitation */
    GenShapedSHBExcitation_fx( shaped_shb_excitation + L_SHB_LAHEAD, lpc_shb, White_exc16k,
                               st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                               coder_type, bwe_exc_extended_16, st_fx->bwe_seed_fx, vf_modified, st_fx->extl_fx,
                               &( st_fx->tbe_demph_fx ), &( st_fx->tbe_premph_fx ), lpc_shb_sf, shb_ener_sf_32,
                               shb_res_gshape, shb_res_dummy, &vind, formant_fac, st_fx->fb_state_lpc_syn_fx,
                               &(st_fx->fb_tbe_demph_fx), &Q_bwe_exc, &Q_bwe_exc_fb,Q_shb, n_mem2, st_fx->prev_Q_bwe_syn, st_fx->total_brate_fx);

    *Q_white_exc = Q_bwe_exc_fb;

    /* rescale the TBE post proc memory */
    FOR( i = 0; i < LPC_SHB_ORDER; i++ )
    {
        st_fx->mem_stp_swb_fx[i] = shl(st_fx->mem_stp_swb_fx[i], Q_bwe_exc - st_fx->prev_Q_bwe_syn);
        move16();
    }

    FOR( i = 0; i < L_FRAME16k; i+=L_SUBFR16k )
    {
        /* TD BWE post-processing */
        PostShortTerm_fx( &shaped_shb_excitation[L_SHB_LAHEAD+i], lpc_shb, &shaped_shb_excitationTemp[i], st_fx->mem_stp_swb_fx,
                          st_fx->ptr_mem_stp_swb_fx, &(st_fx->gain_prec_swb_fx), st_fx->mem_zero_swb_fx, formant_fac );
    }
    Copy( shaped_shb_excitationTemp, &shaped_shb_excitation[L_SHB_LAHEAD], L_FRAME16k );   /* Q_bwe_exc */

    tmp = sub(shl(Q_bwe_exc, 1), 31+16);
    prev_pow = L_shl(FL2WORD32_SCALE(0.00001f, -16), tmp);   /* 2*(Q_bwe_exc) */
    curr_pow = L_shl(FL2WORD32_SCALE(0.00001f, -16), tmp);   /* 2*(Q_bwe_exc) */
    FOR( i = 0; i < L_SHB_LAHEAD; i++ )
    {
        prev_pow = L_mac0( prev_pow, shaped_shb_excitation[i], shaped_shb_excitation[i] ); /*2*Q_bwe_exc*/
        curr_pow = L_mac0( curr_pow, shaped_shb_excitation[i + L_SHB_LAHEAD], shaped_shb_excitation[i+L_SHB_LAHEAD] ); /* 2*Q_bwe_exc */
    }

    if( sub( voice_factors[0], FL2WORD16(0.75f) ) > 0 )
    {
        curr_pow = L_shr( curr_pow, 2 ); /* Q(2*Q_bwe_exc) */
    }

    Lscale = root_a_over_b_fx( curr_pow,
                               shl(Q_bwe_exc, 1),
                               prev_pow,
                               shl(Q_bwe_exc, 1),
                               &exp );

    FOR( i = 0; i < L_SHB_LAHEAD - 1; i++ )
    {
        L_tmp = Mult_32_16( Lscale, shaped_shb_excitation[i] );     /* Q_bwe_exc + (31-exp) - 15 */
        shaped_shb_excitation[i] = round_fx( L_shl( L_tmp, exp ) ); /* Q_bwe_exc */
    }

    Lscale = root_a_fx( Lscale, 31 - exp, &exp );
    L_tmp = Mult_32_16( Lscale, shaped_shb_excitation[L_SHB_LAHEAD - 1] );      /* Q_bwe_exc + (31-exp) - 15 */
    shaped_shb_excitation[L_SHB_LAHEAD - 1] = round_fx( L_shl( L_tmp, exp ) );  /* Q_bwe_exc */

    /* Update SHB excitation */
    Copy( shaped_shb_excitation + L_FRAME16k, st_fx->state_syn_shbexc_fx, L_SHB_LAHEAD );    /* Q_bwe_exc */

    l_subframe_fx = L_FRAME16k/NUM_SHB_SUBGAINS;
    L_ener = 0;
    FOR(i = 0; i < NUM_SHB_SUBGAINS; i++)
    {
        L_tmp = 0;
        ener_tmp[i] = 0;
        FOR(j = 0; j < l_subframe_fx; j++)
        {
            L_tmp = L_mac0( L_tmp, shaped_shb_excitation[i*l_subframe_fx+j], shaped_shb_excitation[i*l_subframe_fx+j] );/* 2*Q_bwe_exc */
        }
        L_tmp = Mult_32_16(L_tmp, FL2WORD16(0.0125));    /* 2*Q_bwe_exc: ener_tmp_fx in (2*Q_bwe_exc) */
        IF( L_tmp != 0 )
        {
            exp = norm_l( L_tmp );
            tmp = extract_h( L_shl( L_tmp, exp ) );
            exp = sub( exp, 30 - (2 * Q_bwe_exc) );

            tmp = div_s( 16384, tmp );
            L_tmp = L_deposit_h( tmp );
            L_tmp = Isqrt_lc( L_tmp, &exp );
            ener_tmp[i] = L_shl( L_tmp, sub(add( exp, shl(Q_bwe_exc,1)),31)); /*2 * Q_bwe_exc: Q31 -exp +exp +2 * Q_bwe_exc -31  */ move32();
            L_ener = L_add(L_ener,L_shr(L_tmp,2));/*29 -exp*/
        }
    }
    ener = s_max(1,round_fx( L_shl( L_ener, sub( exp, 12 ) ) )); /* Q3: 29 -exp +exp -(10+"2") -16 */

    /* WB/SWB bandwidth switching */
    IF( st_fx->bws_cnt_fx > 0 )
    {
        ener = mult(ener, 11587);
        /*bandwidth switching should be updated*/
        if( sub( st_fx->tilt_swb_fx, 16384 ) > 0 )
        {
            st_fx->prev_fractive_fx = 1;
            move16();
        }

        IF( is_fractive == 0 )
        {
            IF( sub( st_fx->tilt_wb_fx, 2048 ) > 0 ) /*assuming st_fx->tilt_wb_fx in Q11*/
            {
                st_fx->tilt_wb_fx = 2048;
                move16();
            }
            ELSE IF( sub( st_fx->tilt_wb_fx, 1024 ) < 0 )
            {
                st_fx->tilt_wb_fx = 1024;
                move16();
            }
            test();
            if( st_fx->prev_fractive_fx == 1 && sub( st_fx->tilt_wb_fx, 1024 ) > 0 )
            {
                st_fx->tilt_wb_fx = 1024;
                move16();
            }
        }
        ELSE
        {
            IF(sub(st_fx->tilt_wb_fx, 8192) > 0)
            {
                IF(st_fx->prev_fractive_fx == 0)
                {
                    st_fx->tilt_wb_fx = 8192;
                }
                ELSE
                {
                    st_fx->tilt_wb_fx = 16384;
                }
            }
            ELSE
            {
                st_fx->tilt_wb_fx = shl(st_fx->tilt_wb_fx, 2);
            }
        }

        IF(ener != 0)
        {
            L_tmp = L_shl(L_mult0(ener, st_fx->tilt_wb_fx), sub(st_fx->Q_syn2, 14));  /* 3+11 +st_fx->Q_syn2 -14 = st_fx->Q_syn2*/
            exp_ener = norm_s(ener);
            tmp = shl(ener, exp_ener);/*Q(3+exp)*/
            inv_ener = div_s(16384, tmp);/*Q(15+14-3-exp) = 26- exp*/

            test();
            IF( L_sub(L_tmp, st_fx->enerLH_fx) > 0) /*st_fx->Q_syn2*/
            {
                st_fx->tilt_wb_fx = extract_h(L_shr(Mult_32_16(st_fx->enerLH_fx, inv_ener), sub(sub(st_fx->Q_syn2, exp_ener),16)));   /*Q11*/
                /*st_fx->Q_syn2 -1 + 26- exp_ener -15 -(st_fx->Q_syn2 -exp_ener -16 ) -16    +1 -1 = (11) *0.5*/
            }
            ELSE IF( L_sub(L_tmp, Mult_32_16(st_fx->enerLH_fx, 1638)) < 0 && sub(is_fractive, 1) == 0 )
            {
                st_fx->tilt_wb_fx = extract_h(L_shr(Mult_32_16(st_fx->enerLH_fx, inv_ener), sub(sub(st_fx->Q_syn2, exp_ener), 15)));    /*Q11*/
                /*st_fx->Q_syn2 -1 + 26- exp_ener -15 -(st_fx->Q_syn2 -exp_ener -15 ) -16 = (11) 0.25*/
            }


            L_tmp = L_mult0(st_fx->prev_ener_shb_fx, inv_ener);  /*Q(3+15+14-3-exp_ener) = 29 -exp_ener*/
            GainFrame_prevfrm_fx = L_shr(L_tmp, sub(11, exp_ener));  /*29 -exp_ener -(8-exp_ener )= Q18*/
        }
        ELSE
        {
            GainFrame_prevfrm_fx = 0;
        }

        IF( sub(is_fractive , 1) == 0 )
        {
            GainFrame = L_shl((Word32)st_fx->tilt_wb_fx, 10);
        }
        ELSE
        {
            GainFrame = L_shl((Word32)st_fx->tilt_wb_fx, 8);
        }

        test();
        IF( sub((is_fractive & st_fx->prev_fractive_fx), 1) == 0 && L_sub(GainFrame, GainFrame_prevfrm_fx) > 0)
        {
            GainFrame = L_add(Mult_32_16(GainFrame_prevfrm_fx, 26214), Mult_32_16(GainFrame, 6554));/* 18 +15 -15 = 18*/
        }
        ELSE
        {
            test();
            test();
            test();
            test();
            IF((L_sub(L_shr(st_fx->prev_enerLH_fx, 1), st_fx->enerLH_fx) < 0 && L_sub(st_fx->prev_enerLH_fx, L_shr(st_fx->enerLH_fx, 1)) > 0)
            && (L_sub(L_shr(st_fx->prev_enerLL_fx, 1), st_fx->enerLL_fx) < 0 && L_sub(st_fx->prev_enerLL_fx, L_shr(st_fx->enerLL_fx, 1)) > 0) && (is_fractive ^ st_fx->prev_fractive_fx) == 0)
            {
                GainFrame = L_add(L_shr(GainFrame, 1), L_shr(GainFrame_prevfrm_fx, 1));
            }
            ELSE
            {
                test();
                IF(is_fractive == 0 && sub(st_fx->prev_fractive_fx, 1) == 0)
                {
                    L_tmp1 = L_shl(Mult_32_16(GainFrame, 3277), 13);   /* 31 */
                    L_tmp = L_sub(2147483647, L_tmp1); /* 31 */
                    GainFrame = L_add(Mult_32_32(GainFrame, L_tmp), Mult_32_32(GainFrame_prevfrm_fx, L_tmp1));  /* 18 */
                }
                ELSE
                {
                    GainFrame = L_add(L_shr(GainFrame, 1), L_shr(L_min(GainFrame_prevfrm_fx, GainFrame), 1)); /* 18 */
                }
            }
        }

        GainFrame = Mult_32_16(GainFrame, i_mult(sub(N_WS2N_FRAMES, st_fx->bws_cnt_fx), 819));  /*Q18*/
    }
    ELSE
    {
        IF(st_fx->bws_cnt1_fx > 0)
        {
            GainFrame = Mult_32_16(GainFrame, i_mult(st_fx->bws_cnt1_fx, 819)); /*Q18*/
        }
        IF(sub(st_fx->nbLostCmpt, 1) >= 0)
        {
            ener = s_max(1, ener);
            exp_ener = norm_s(ener);
            tmp = shl(ener, exp_ener);/*Q(3+exp)*/
            inv_ener = div_s(16384, tmp);/*Q(15+14-3-exp)*/
            prev_ener_ratio_fx = L_shr(L_mult0(st_fx->prev_ener_shb_fx, inv_ener), sub(11, exp_ener));  /*Q: 3+ 26 -exp -11 + exp = 18 */
        }

        IF(sub(st_fx->nbLostCmpt, 1) == 0)
        {
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF( sub(st_fx->clas_dec, UNVOICED_CLAS) != 0 && sub(st_fx->clas_dec, UNVOICED_TRANSITION) != 0 && sub(st_fx->tilt_swb_fec_fx, 16384) < 0 &&
            ((L_sub(st_fx->enerLL_fx, L_shr(st_fx->prev_enerLL_fx, 1)) > 0 && L_sub(L_shr(st_fx->enerLL_fx, 1), st_fx->prev_enerLL_fx) < 0)|| (L_sub(st_fx->enerLH_fx, L_shr(st_fx->prev_enerLH_fx, 1)) > 0 && L_sub(L_shr(st_fx->enerLH_fx, 1), st_fx->prev_enerLH_fx) < 0)))
            {
                IF(L_sub(L_shr(prev_ener_ratio_fx, 2), GainFrame) > 0) /*18*/
                {
                    GainFrame = L_add(Mult_32_16(prev_ener_ratio_fx, 13107), Mult_32_16(GainFrame, 19661));/*18*/
                }
                ELSE IF(L_sub(L_shr(prev_ener_ratio_fx, 1), GainFrame) > 0)
                {
                    GainFrame = L_add(Mult_32_16(prev_ener_ratio_fx, 26214), Mult_32_16(GainFrame, 6554));
                }
                ELSE
                {
                    GainFrame = L_add(Mult_32_16(prev_ener_ratio_fx, 6554), Mult_32_16(GainFrame, 26214));
                }

                test();
                IF(sub(tilt_swb_fec, st_fx->tilt_swb_fec_fx) > 0 && st_fx->tilt_swb_fec_fx > 0)
                {
                    exp = norm_s(st_fx->tilt_swb_fec_fx);
                    tmp = shl(st_fx->tilt_swb_fec_fx, exp);/*Q(11+exp)*/
                    tmp = div_s(16384, tmp);/*Q(15+14-11-exp)*/
                    tmp = extract_h(L_shl(L_mult0(tmp, st_fx->tilt_wb_fx),sub(exp,1)));/*18 -exp +11  + exp -1 -16  =12;          */
                    GainFrame = L_shl(Mult_32_16(GainFrame, s_min(tmp, 20480)), 3); /*Q18 = 18 +12 -15 +3 */
                }

            }
            ELSE IF( (sub(st_fx->clas_dec, UNVOICED_CLAS) != 0 || sub(st_fx->tilt_swb_fec_fx, 16384) > 0) && L_sub(L_shr(prev_ener_ratio_fx, 2), GainFrame) > 0 &&
                     (L_sub(st_fx->enerLL_fx, L_shr(st_fx->prev_enerLL_fx, 1)) > 0 || L_sub(st_fx->enerLH_fx, L_shr(st_fx->prev_enerLH_fx, 1)) > 0) )
            {
                GainFrame = L_add(Mult_32_16(prev_ener_ratio_fx, 6554), Mult_32_16(GainFrame, 26214));
            }
        }
        ELSE IF( sub(st_fx->nbLostCmpt, 1) > 0 )
        {
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF(L_sub(L_shr(prev_ener_ratio_fx, 2), GainFrame) > 0 && ((sub(st_fx->codec_mode, MODE1) == 0 && L_sub(st_fx->enerLL_fx, st_fx->prev_enerLL_fx) > 0 && L_sub(st_fx->enerLH_fx, st_fx->prev_enerLH_fx) > 0) || sub(st_fx->codec_mode, MODE2) == 0))
            {
                test();
                IF( sub(tilt_swb_fec, 20480) > 0 && sub(st_fx->tilt_swb_fec_fx, 20480) > 0 )
                {
                    GainFrame = L_min(L_add(Mult_32_16(prev_ener_ratio_fx, 26214), Mult_32_16(GainFrame, 6554)), L_shl(Mult_32_16(GainFrame, 16384), 3));  /*Q18*/
                }
                ELSE
                {
                    GainFrame = L_min(L_add(Mult_32_16(prev_ener_ratio_fx, 16384), Mult_32_16(GainFrame, 16384)), L_shl(Mult_32_16(GainFrame, 16384), 3));  /*Q18*/
                }
            }
            ELSE IF(L_sub(prev_ener_ratio_fx, GainFrame) > 0 &&((sub(st_fx->codec_mode, MODE1) == 0 && L_sub(st_fx->enerLL_fx, st_fx->prev_enerLL_fx) > 0 && L_sub(st_fx->enerLH_fx, st_fx->prev_enerLH_fx) > 0) || sub(st_fx->codec_mode, MODE2) == 0))
            {
                test();
                IF( sub(tilt_swb_fec, 20480) > 0 && sub(st_fx->tilt_swb_fec_fx, 20480) > 0 )
                {
                    GainFrame = L_add(Mult_32_16(prev_ener_ratio_fx, 16384), Mult_32_16(GainFrame, 16384));
                }
                ELSE
                {
                    GainFrame = L_add(Mult_32_16(prev_ener_ratio_fx, 6554), Mult_32_16(GainFrame, 26214));
                }
            }
        }
    }
    st_fx->prev_fractive_fx = is_fractive;
    move16();

    /* Adjust the subframe and frame gain of the synthesized shb signal */
    IF( sub(st_fx->L_frame_fx, L_FRAME) == 0 )
    {
        /* pitch = 0.25f*sum_s(pitch_buf, 4); */
        L_tmp = L_mult(pitch_buf[0], 8192);
        FOR (i=1; i<NB_SUBFR; i++)
        {
            L_tmp = L_mac(L_tmp, pitch_buf[i],8192); /* pitch_buf in Q6 x 0.25 in Q15 */
        }
        pitch_fx = round_fx(L_tmp); /* Q6 */
    }
    ELSE
    {
        /* pitch_fx = 0.2f*sum_s(pitch_buf, 5); */
        L_tmp = L_mult(pitch_buf[0], 6554);
        FOR (i=1; i<NB_SUBFR16k; i++)
        {
            L_tmp = L_mac(L_tmp, pitch_buf[i],6554); /* pitch_buf in Q6 x 0.2 in Q15 */
        }
        pitch_fx = round_fx(L_tmp); /* Q6 */
    }

    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF( ((L_sub(st_fx->total_brate_fx, ACELP_24k40) >= 0 && sub(st_fx->prev_coder_type_fx, coder_type) == 0 && sub(coder_type, UNVOICED) != 0)
         || (L_sub(st_fx->total_brate_fx, ACELP_16k40) <= 0 && (sub(st_fx->prev_coder_type_fx, coder_type) == 0 || (sub(st_fx->prev_coder_type_fx, VOICED) == 0  && sub(coder_type, GENERIC) == 0) || (sub(st_fx->prev_coder_type_fx, GENERIC) == 0  && sub(coder_type, VOICED) == 0))))
        && sub(pitch_fx, 4480 /*70 in Q6*/) > 0 && sub(st_fx->extl_fx, FB_TBE) < 0)
    {
        FOR(i=0; i<NUM_SHB_SUBGAINS; i++)
        {
            GainShape_tmp[i] = GainShape[shl(i, 2)]; /* Q15 */  move16();
        }

        FOR(i=0; i<NUM_SHB_SUBGAINS; i++)
        {
            /* if( ener_tmp_fx[i]*GainShape_tmp_fx[i] > st_fx->prev_ener_fx*st_fx->prev_GainShape_fx ) */
            L_tmp1 = Mult_32_16(ener_tmp[i], GainShape_tmp[i]); /* (2*Q_bwe_exc) */
            L_tmp2 = Mult_32_16(st_fx->prev_ener_fx, st_fx->prev_GainShape_fx); /* (2*st_fx->prev_ener_fx_Q) */
            tmp = sub(shl(Q_bwe_exc, 1), shl(st_fx->prev_ener_fx_Q, 1));
            L_tmp2 = L_shl(L_tmp2, tmp); /* new Q = (2*Q_bwe_exc) */

            IF (L_sub(L_tmp1,L_tmp2) > 0)
            {
                /*GainShape_tmp_fx[i] = 0.5f*(L_tmp2/ener_tmp_fx[i] + GainShape_tmp_fx[i]);*/
                /* tmp = L_tmp2/ener_tmp_fx[i]*/
                L_tmp = L_tmp2;
                if(L_tmp2 < 0)
                {
                    L_tmp = L_negate(L_tmp2);
                }

                expb = norm_l(L_tmp);
                fracb = round_fx(L_shl(L_tmp,expb));
                expb = 30-expb; /* - (2*Q_bwe_exc_ext);                    */

                expa = norm_l(ener_tmp[i]);
                fraca = extract_h(L_shl(ener_tmp[i],expa));
                expa = 30-expa ;

                scale = shr(sub(fraca,fracb),15);
                fracb = shl(fracb,scale);
                expb = sub(expb,scale);

                tmp = div_s(fracb,fraca);
                exp = sub(expb,expa);
                L_tmp = L_shl(tmp, add(exp, 16));

                tmp = round_fx(L_tmp);
                tmp = add(tmp, GainShape_tmp[i]);
                GainShape_tmp[i] = extract_h(L_mult(tmp,16384));
                move16(); /* Q15 */
            }

            st_fx->prev_ener_fx = ener_tmp[i];
            st_fx->prev_GainShape_fx = GainShape_tmp[i];
            st_fx->prev_ener_fx_Q = Q_bwe_exc;
        }
        FOR(i=0; i<NUM_SHB_SUBFR; i++)
        {
            GainShape[i] = GainShape_tmp[i*NUM_SHB_SUBGAINS/NUM_SHB_SUBFR];
            move16();
        }
    }
    ELSE
    {
        st_fx->prev_ener_fx_Q = Q_bwe_exc;
        move16();
    }


    /* Back up the Q_bwe_exc associated with shaped_shb_excitation for the next frame*/
    st_fx->prev_Q_bwe_syn = Q_bwe_exc;
    move16();

    /* Scale the shaped excitation */
    ScaleShapedSHB_fx( SHB_OVERLAP_LEN,
                       shaped_shb_excitation,   /* i/o: Q_bwe_exc */
                       st_fx->syn_overlap_fx,
                       GainShape,   /* Q15  */
                       GainFrame,   /* Q18  */
                       window_shb_fx,
                       subwin_shb_fx,
                       &Q_bwe_exc
                       , &Qx
                       , n_mem3
                       , st_fx->prev_Q_bwe_syn2
                     );
    /* i: GainShape Q15                         */
    /* i: GainFrame Q18                         */
    /* i: shaped_shb_excitation Q_bwe_exc       */
    /* o: shaped_shb_excitation Q_bwe_exc       */
    /* o: st_fx->syn_overlap_fx Q_bwe_exc       */


    max = 0;
    FOR( i = 0;  i < L_FRAME16k; i++ )
    {
        max = s_max( max, shaped_shb_excitation[i] ); /* Q0 */
    }

    IF( max == 0 )
    {
        curr_frame_pow = 0;
        move16();
        n = 0;
        move16();
    }
    ELSE
    {
        n = norm_s( max );
        max = 0;
        FOR( i = 0;  i < L_FRAME16k; i++ )
        {
            shaped_shb_excitation_frac[i] = shl( shaped_shb_excitation[i], n ); /*Q_bwe_exc+n*/ move16();
        }

        curr_frame_pow = 0;
        FOR( i = 0;  i < L_FRAME16k; i++ )
        {
            L_tmp = L_mult0( shaped_shb_excitation_frac[i], shaped_shb_excitation_frac[i] ); /*2*(Q_bwe_exc+n)*/
            curr_frame_pow = L_add( curr_frame_pow, L_shr( L_tmp, 9 ) ); /*2*(Q_bwe_exc+n)-9*/
        }
    }
    curr_frame_pow_exp = sub(shl(add( Q_bwe_exc, n ), 1), 9);

    IF( sub( st_fx->prev_frame_pow_exp, curr_frame_pow_exp ) > 0 )
    {
        st_fx->prev_swb_bwe_frame_pow_fx = L_shr( st_fx->prev_swb_bwe_frame_pow_fx,
                                           sub(st_fx->prev_frame_pow_exp, curr_frame_pow_exp ) );
        move32();
        st_fx->prev_frame_pow_exp = curr_frame_pow_exp;
        move16();

    }
    ELSE
    {
        curr_frame_pow = L_shr( curr_frame_pow, sub( curr_frame_pow_exp, st_fx->prev_frame_pow_exp ) );
        curr_frame_pow_exp = st_fx->prev_frame_pow_exp;
    }

    test();
    IF( !st_fx->bfi_fx && st_fx->prev_bfi_fx )
    {
        L_tmp = L_shr( curr_frame_pow, 4 );
        L_tmp = Mult_32_16( L_tmp, 17476 );

        test();
        test();
        IF( ( L_sub( L_shr( curr_frame_pow, 1 ), st_fx->prev_swb_bwe_frame_pow_fx ) > 0 ) &&
            ( L_sub( st_fx->prev_swb_bwe_frame_pow_fx, L_tmp ) > 0 ) && sub(st_fx->prev_coder_type_fx,UNVOICED)  == 0)
        {
            L_tmp = root_a_over_b_fx( st_fx->prev_swb_bwe_frame_pow_fx, curr_frame_pow_exp, curr_frame_pow, curr_frame_pow_exp, &exp );
            scale = round_fx( L_shl( L_tmp, exp ) ); /*Q15*/

            L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
            L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
            L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
            temp = round_fx( L_shl( L_tmp, exp ) ); /*Q15*/
        }
        ELSE
        {
            scale = temp = 32767;
            move16();/*Q15*/
        }
        FOR( j = 0; j < 8; j++ )
        {
            GainShape[2 * j] = mult_r( GainShape[2 * j], scale );
            move16();
            GainShape[2 * j + 1] = mult_r( GainShape[2 * j + 1], scale );
            move16();
            FOR( i = 0; i < L_FRAME16k / 8; i++ )
            {
                shaped_shb_excitation[i + j*L_FRAME16k/8] = mult_r( shaped_shb_excitation[i + j*L_FRAME16k/8],scale );
                move16();
            }

            IF( temp > 0 )
            {
                IF( sub( scale, temp ) < 0 )
                {
                    scale = div_s( scale, temp );
                }
                ELSE
                {
                    scale = 32767;
                    move16();
                }
            }
            ELSE
            {
                scale = 0;
                move16();
            }
        }
    }

    /* adjust the FEC frame energy */
    IF( st_fx->bfi_fx )
    {
        scale = temp = 4096;
        move16();/*Q12*/

        IF (sub(st_fx->nbLostCmpt,1) == 0 )
        {
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            test();
            IF( L_sub(curr_frame_pow, st_fx->prev_swb_bwe_frame_pow_fx) >0  &&
                sub(st_fx->prev_coder_type_fx, UNVOICED)!= 0 &&
                sub(st_fx->last_good_fx,UNVOICED_CLAS) != 0 )
            {
                L_tmp = root_a_over_b_fx( st_fx->prev_swb_bwe_frame_pow_fx, curr_frame_pow_exp, curr_frame_pow, curr_frame_pow_exp, &exp ); /*31 - exp*/
                scale = round_fx( L_shl( L_tmp, sub(exp,3))); /*Q12*/
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                temp = round_fx( L_shl( L_tmp, sub(exp,3))); /*Q12*/
            }
            ELSE IF( L_sub(curr_frame_pow,  L_shr(st_fx->prev_swb_bwe_frame_pow_fx,1)) <0 && sub(st_fx->nbLostCmpt,1) == 0 &&
                     (L_sub(st_fx->enerLL_fx, L_shr( st_fx->prev_enerLL_fx,1)) > 0 || L_sub(st_fx->enerLH_fx, L_shr(st_fx->prev_enerLH_fx,1)) >0 ) &&
                     (sub(st_fx->prev_coder_type_fx ,UNVOICED) == 0 || sub(st_fx->last_good_fx, UNVOICED_CLAS) == 0 || sub(st_fx->tilt_swb_fec_fx , 10240) > 0))
            {
                L_tmp = root_a_over_b_fx( st_fx->prev_swb_bwe_frame_pow_fx, curr_frame_pow_exp, curr_frame_pow, curr_frame_pow_exp, &exp );
                scale = round_fx( L_shl( L_tmp, sub(exp,3) ) ); /*Q12*/
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                temp = round_fx( L_shl( L_tmp, sub(exp,3) ) ); /*Q12*/
            }
        }
        ELSE IF (sub(st_fx->nbLostCmpt,1) > 0 )
        {
            test();
            test();
            test();
            test();
            test();
            IF( L_sub(curr_frame_pow , st_fx->prev_swb_bwe_frame_pow_fx) >0  )
            {
                L_tmp = root_a_over_b_fx( st_fx->prev_swb_bwe_frame_pow_fx, curr_frame_pow_exp, curr_frame_pow, curr_frame_pow_exp, &exp );
                scale = round_fx( L_shl( L_tmp, sub(exp,3) ) ); /*Q12*/
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                temp = round_fx( L_shl( L_tmp, sub(exp,3) ) ); /*Q12*/

            }
            ELSE IF( curr_frame_pow < 0.5f *st_fx->prev_swb_bwe_frame_pow_fx &&
                     (st_fx->enerLL_fx > 0.5 * st_fx->prev_enerLL_fx || st_fx->enerLH_fx > 0.5 *st_fx->prev_enerLH_fx) &&
                     (st_fx->prev_coder_type_fx == UNVOICED || st_fx->last_good_fx == UNVOICED_CLAS || st_fx->tilt_swb_fec_fx > 5.0f) )
            {
                L_tmp = root_a_over_b_fx( st_fx->prev_swb_bwe_frame_pow_fx, curr_frame_pow_exp, curr_frame_pow, curr_frame_pow_exp, &exp );
                L_tmp =L_min(L_tmp,L_shl(3,(31 - exp)));/*31 - exp*/
                scale = round_fx( L_shl( L_tmp, sub(exp,3) )); /*Q12*/
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                L_tmp = root_a_fx( L_tmp, 31 - exp, &exp );
                temp = round_fx( L_shl( L_tmp, sub(exp,3) ) ); /*Q12*/

            }
        }
        FOR( j = 0; j < 8; j++ )
        {
            GainShape[2 * j] = shl(mult_r( GainShape[2 * j], scale ),3);
            move16(); /* 15 +12 +3-15 =15*/
            GainShape[2 * j + 1] = shl(mult_r( GainShape[2 * j + 1], scale ),3);
            move16();
            FOR( i = 0; i < 40; i++ )
            {
                shaped_shb_excitation[add(i,i_mult(j,40))] = shl(mult_r( shaped_shb_excitation[add(i,i_mult(j,40))], scale) ,3);
                move16(); /* Q_bwe_exc +12+3 -15  =Q_bwe_exc*/
            }

            IF( temp > 0 )
            {
                IF( sub( scale, temp ) < 0 )
                {
                    scale = shr(div_s( scale, temp ),3);
                }
                ELSE
                {
                    scale = 4096;
                    move16();
                }
            }
            ELSE
            {
                scale = 0;
                move16();
            }
        }
    }

    st_fx->prev_swb_bwe_frame_pow_fx = curr_frame_pow;
    move32();
    st_fx->prev_frame_pow_exp = curr_frame_pow_exp;
    move16();

    L_prev_ener_shb = L_deposit_l(0);
    FOR( i = 0; i < L_FRAME16k; i++ )
    {
        L_prev_ener_shb = L_mac0( L_prev_ener_shb, shaped_shb_excitation[i], shaped_shb_excitation[i] ); /* Q0 */
    }

    /* st->prev_ener_shb = sqrt(st->prev_ener_shb/L_FRAME16k) */
    L_prev_ener_shb = Mult_32_16( L_prev_ener_shb, 26214 ); /* 2*Q_bwe_exc_mod+8; 26214=(1/L_FRAME16k) in Q23 */
    st_fx->prev_ener_shb_fx = 0;
    move16();

    IF( L_prev_ener_shb != 0 )
    {
        exp = norm_l( L_prev_ener_shb );
        tmp = extract_h( L_shl( L_prev_ener_shb, exp ) );
        exp = sub( exp, 30 - ( 2 * Q_bwe_exc + 8 ) );

        tmp = div_s( 16384, tmp );
        L_tmp = L_deposit_h( tmp );
        L_tmp = Isqrt_lc( L_tmp, &exp );

        st_fx->prev_ener_shb_fx = round_fx( L_shl( L_tmp, sub( exp, 12 ) ) ); /* Q3 */
    }
    /* st->prev_SWB_fenv[i] = sqrt(curr_frame_pow/L_FRAME16k); */
    L_tmp = Mult_32_16( curr_frame_pow, 26214 ); /* curr_frame_pow_exp+8; 26214=(1/L_FRAME16k) in Q23 */
    tmp =0;
    IF( L_tmp != 0 )
    {
        exp = norm_l( L_tmp );
        tmp = extract_h( L_shl( L_tmp, exp ) );
        exp = sub( exp, 30 - ( curr_frame_pow_exp + 8 ) );

        tmp = div_s( 16384, tmp );
        L_tmp = L_deposit_h( tmp );
        L_tmp = Isqrt_lc( L_tmp, &exp );
        tmp = round_fx( L_shl( L_tmp, sub( exp, 14 ) ) ); /* Q1 */
    }
    set16_fx(st_fx->prev_SWB_fenv_fx, tmp, SWB_FENV); /* Q1 */


    /* rescale the memories if Q_bwe_exc is different from previous frame */
    sc = sub( Q_bwe_exc, st_fx->prev_Q_bwe_syn2 );
    IF( sc != 0 )
    {
        FOR( i = 0; i < (2*ALLPASSSECTIONS_STEEP+1); i++ )
        {
            st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx[i] = shl( st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx[i], sc );
            move16();
        }

        IF(sub(st_fx->L_frame_fx, L_FRAME) == 0)
        {
            FOR( i = 0; i < HILBERT_MEM_SIZE; i++ )
            {
                st_fx->genSHBsynth_Hilbert_Mem_fx[i] = L_shl( st_fx->genSHBsynth_Hilbert_Mem_fx[i], sc );
                move32();
            }
        }
        IF( st_fx->output_Fs_fx == 48000 )
        {
            Scale_sig(st_fx->int_3_over_2_tbemem_dec_fx, INTERP_3_2_MEM_LEN, sc);
        }
    }
    /* i: shaped_shb_excitation[320] in (Q_bwe_exc)            */
    /* i/o: st_fx->genSHBsynth_Hilbert_Mem_fx in (Q_bwe_exc)   */
    /* i/o: st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx in (Q_bwe_exc)   */
    /* o: error in (Qx)                                 */
    GenSHBSynth_fx( shaped_shb_excitation, error, st_fx->genSHBsynth_Hilbert_Mem_fx,
                    st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx, st_fx->L_frame_fx, &( st_fx->syn_dm_phase_fx ) );
    Copy( error, st_fx->old_tbe_synth_fx, L_FRAME32k );

    /* resample SHB synthesis (if needed) and scale down */
    synth_scale_fx = 32767;
    move16(); /* 1.0 in Q15 */
    if(sub(st_fx->codec_mode,MODE1)==0)
    {
        synth_scale_fx = 29491;
        move16(); /* 0.9 in Q15 */
    }

    IF( L_sub(st_fx->output_Fs_fx, 48000) == 0 )
    {
        IF( L_sub(st_fx->extl_fx,FB_TBE) == 0 )
        {
            tmp = norm_l( GainFrame );
            if(GainFrame == 0)
            {
                tmp = 31;
            }
            L_tmp = L_shl(GainFrame,tmp);/* 18 + tmp */

            tmp1 =0;
            FOR( i = 0; i < L_FRAME16k; i++ )
            {
                L_tmp1 = Mult_32_16( L_tmp, GainShape[NUM_SHB_SUBFR * i / L_FRAME16k] ); /* Q : 18 + tmp +15 -15*/
                White_exc16k[i] = round_fx( Mult_32_16( L_tmp1, White_exc16k[i] ) );/* 18 + tmp +*Q_white_exc -15 -16 */
                tmp1 =s_max(tmp1,White_exc16k[i]);
            }

            *Q_white_exc = sub(add(*Q_white_exc, tmp),13); /* *Q_white_exc + 18 + tmp -15 -16 */
            tmp = norm_s( tmp1 );
            if(tmp1 == 0)
            {
                tmp = 15;
            }

            FOR(i=0; i<L_FRAME16k; i++)
            {
                White_exc16k[i] = shl(White_exc16k[i],tmp);/* (*Q_white_exc + tmp)*/  move16();
                White_exc16k[i] = shr(White_exc16k[i],1);
                move16();
            }
            *Q_white_exc = sub(add(*Q_white_exc, tmp),1);
            move16();
        }

        IF( sub(synth_scale_fx,32767) != 0 ) /* 1.0 in Q15 */
        {
            FOR( i=0; i<L_FRAME32k; i++ )
            {
                error[i] = mult_r(error[i], synth_scale_fx);
                move16();
            }
        }

        interpolate_3_over_2_allpass_fx( error, L_FRAME32k, synth, st_fx->int_3_over_2_tbemem_dec_fx, allpass_poles_3_ov_2 );
    }
    ELSE IF( L_sub(st_fx->output_Fs_fx, 32000) == 0 )
    {
        IF( sub(synth_scale_fx,32767) != 0 ) /* 1.0 in Q15 */
        {
            FOR( i = 0; i < L_FRAME32k; i++ )
            {
                synth[i] = mult_r( synth_scale_fx, error[i] );
                move16();/*Qx*/
            }
        }
        ELSE
        {
            Copy(error, synth, L_FRAME32k);
        }
    }
    ELSE IF( L_sub(st_fx->output_Fs_fx, 16000) == 0 )
    {
        IF( sub(synth_scale_fx,32767) != 0 ) /* 1.0 in Q15 */
        {
            FOR( i = 0; i < L_FRAME32k; i++ )
            {
                error[i] = mult_r(error[i],synth_scale_fx);
                move16();
            }
        }

        Decimate_allpass_steep_fx( error, st_fx->mem_resamp_HB_32k_fx, L_FRAME32k, synth );
    }
    ELSE IF( L_sub(st_fx->output_Fs_fx, 8000) == 0 )
    {
        /* set high band synthesis to zero when NB is output; This will allow the swb synthesis to be carried out and
            subsequently populate all memories for use when the codec switches back to output_fs = 32kHz */
        set16_fx( synth, 0, L_FRAME32k );
        Q_bwe_exc = 31;
    }

    /* Update previous frame parameters for FEC */
    Copy( lsf_shb, st_fx->lsp_prevfrm_fx, LPC_SHB_ORDER );
    IF(sub(st_fx->codec_mode, MODE1) == 0)
    {
        st_fx->GainFrame_prevfrm_fx = GainFrame;
        move16(); /*Q18*/
        st_fx->tilt_swb_fec_fx = tilt_swb_fec;

        if( !st_fx->bfi_fx )
        {
            st_fx->GainAttn_fx = 32767;
            move16();
        }
    }
    ELSE
    {
        IF( !st_fx->bfi_fx )
        {
            st_fx->GainFrame_prevfrm_fx = GainFrame;
            move16(); /*Q18*/
            st_fx->tilt_swb_fec_fx = tilt_swb_fec;
            move16();
            st_fx->GainAttn_fx = 32767;
            move16();
        }
    }

    st_fx->prev_ener_fx = ener_tmp[NUM_SHB_SUBGAINS-1];
    st_fx->prev_GainShape_fx = GainShape[NUM_SHB_SUBFR-1];
    *Q_synth = Q_bwe_exc;
    move16();
    st_fx->prev_Q_bwe_syn2 = Q_bwe_exc;
    move16();
    st_fx->prev_Qx = Q_bwe_exc;
    move16();
    return;
}

static void gradientGainShape(
    Decoder_State_fx *st_fx,
    Word16 *GainShape,
    Word32 *GainFrame)
{
    Word16 i,j,tmp;
    Word16 GainShapeTemp[NUM_SHB_SUBFR/4];
    Word16 GainGrad0[3];
    Word16 GainGrad1[3];
    Word16 GainGradFEC[4];

    /* the previous frame gainshape gradient and the gainshape gradient pattern for the current frame */
    FOR( j = 0; j < 3; j++ )
    {
        GainGrad0[j] = sub( shr( st_fx->GainShape_Delay[j + 1], 1 ), shr( st_fx->GainShape_Delay[j], 1 ) );
        move16();/* Q14 */
        GainGrad1[j] = sub( shr( st_fx->GainShape_Delay[j + 5], 1 ), shr( st_fx->GainShape_Delay[j + 4], 1 ) );
        move16();/* Q14 */
        GainGradFEC[j + 1] = add( mult_r( GainGrad0[j], 13107 ), mult_r( GainGrad1[j], 19660 ) );
        move16(); /* Q14 */
    }

    /* gradient for the first gainshape */
    test();
    test();
    test();
    IF( ( ( sub( shr( GainGrad1[2], 1 ), GainGrad1[1] ) > 0 ) && ( sub( shr( GainGrad1[1], 1 ), GainGrad1[0] ) > 0 ) ) ||
        ( ( sub( shr( GainGrad1[2], 1 ), GainGrad1[1] ) < 0 ) && ( sub( shr( GainGrad1[1], 1 ), GainGrad1[0] ) < 0 ) ) )
    {
        GainGradFEC[0] = add( mult_r( GainGrad1[1], 3277 ), mult_r( GainGrad1[2], 29490 ) );
        move16(); /* Q14 */
    }
    ELSE
    {
        GainGradFEC[0] = add( mult_r( GainGrad1[0], 6553 ), mult_r( GainGrad1[1], 9830 ) );
        move16();
        GainGradFEC[0] = add( GainGradFEC[0], mult_r( GainGrad1[2], 16384 ) );
        move16(); /* Q14 */
    }

    /* get the first gainshape template */
    test();
    test();
    IF( ( st_fx->prev_coder_type_fx == UNVOICED || st_fx->last_good_fx == UNVOICED_CLAS ) && GainGradFEC[0] > 0 )
    {
        GainShapeTemp[0] = add( shr( st_fx->GainShape_Delay[7], 1 ), GainGradFEC[0] );
        move16();
    }
    ELSE IF( GainGradFEC[0] > 0 )
    {
        GainShapeTemp[0] = add( shr( st_fx->GainShape_Delay[7], 1 ), mult_r( GainGradFEC[0], 16384 ) );
        move16(); /* Q14 */
    }
    ELSE
    {
        GainShapeTemp[0] = shr( st_fx->GainShape_Delay[7], 1 );
        move16();/* Q14 */
    }

    /*Get the second the third and the fourth gainshape template*/

    tmp = shr( GainGrad1[2], 3 );   /* GainGrad1[2]/8 */
    tmp = mult_r( tmp, 26214 );     /* 0.8 in Q15 tmp*(8/10) */

    test();
    IF( ( sub( tmp, GainGrad1[1] ) > 0 ) && GainGrad1[1] > 0 )
    {
        FOR( i = 1; i < NUM_SHB_SUBFR / 4; i++ )
        {
            GainShapeTemp[i] = add( GainShapeTemp[i - 1], mult_r( GainGradFEC[i], 26214 ) );
            move16(); /* GainShapeTemp[i-1] + 0.8* GainShapeTemp[i] */
            GainShapeTemp[i] = s_max( GainShapeTemp[i], FL2WORD16(0.01f) );
            move16();
        }
    }
    ELSE
    {
        test();
        IF( ( sub( tmp, GainGrad1[1] ) > 0 ) && GainGrad1[1] < 0 )
        {
            FOR( i = 1; i < NUM_SHB_SUBFR / 4; i++ )
            {
                GainShapeTemp[i] = add( GainShapeTemp[i - 1], mult_r( GainGradFEC[i], 6553 ) );
                move16(); /* GainShapeTemp[i-1] + 0.8* GainShapeTemp[i] */
                GainShapeTemp[i] = s_max( GainShapeTemp[i], FL2WORD16(0.01f) );
                move16(); /* Q14 */
            }
        }
        ELSE
        {
            FOR( i = 1; i < NUM_SHB_SUBFR / 4; i++ )
            {
                GainShapeTemp[i] = add( GainShapeTemp[i - 1], GainGradFEC[i] );
                move16();
                GainShapeTemp[i] = s_max( GainShapeTemp[i], FL2WORD16(0.01f) );
                move16();
            }
        }
    }

    /* Get the gainshape and gain frame for the current frame*/
    test();
    test();
    test();
    IF( ( st_fx->prev_coder_type_fx == UNVOICED || st_fx->last_good_fx == UNVOICED_CLAS ) &&  st_fx->nbLostCmpt == 1 )
    {
        FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
        {
            FOR( j = 0; j < 4; j++ )
            {
                tmp = mult_r( GainShapeTemp[i], 19660 ); /* GainShapeTemp[i]*0.6 */

                IF( sub( 8192, tmp ) > 0 )
                {
                    GainShape[i * 4 + j ] = shl( tmp, 2 );
                    move16(); /* (GainShapeTemp[i]*0.6)>>1 */
                }
                ELSE
                {
                    GainShape[i * 4 + j ] = 32767;
                    move16(); /* Clipping here to avoid the a huge change of the code due to gain shape change */
                }
            }
        }
        st_fx->GainAttn_fx = mult_r( st_fx->GainAttn_fx, 31129 );
    }
    ELSE IF( st_fx->prev_coder_type_fx == UNVOICED || st_fx->last_good_fx == UNVOICED_CLAS )
    {
        FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
        {
            FOR( j = 0; j < 4; j++ )
            {
                IF( sub( GainShapeTemp[i], 16384 ) < 0 )
                {
                    GainShape[i * 4 + j ] = shl( GainShapeTemp[i], 1 );
                    move16();
                }
                ELSE
                {
                    GainShape[i * 4 + j ] = 32767 ;
                    move16();
                }
            }
        }
        st_fx->GainAttn_fx = mult_r( st_fx->GainAttn_fx, 31129 );
    }
    ELSE IF( st_fx->nbLostCmpt > 1 )
    {
        FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
        {
            FOR( j = 0; j < 4; j++ )
            {
                GainShape[i * 4 + j ] = GainShapeTemp[i];
                move16();
            }
        }
        st_fx->GainAttn_fx = mult_r( st_fx->GainAttn_fx, 16384 );
    }
    ELSE
    {
        FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
        {
            FOR( j = 0; j < 4; j++ )
            {
                IF( sub( GainShapeTemp[i], 16384 ) < 0 )
                {
                    GainShape[i * 4 + j] = shl( GainShapeTemp[i], 1 );
                    move16();
                }
                ELSE
                {
                    GainShape[i * 4 + j] = 32767;
                    move16();
                }
            }
        }
        st_fx->GainAttn_fx = mult_r( st_fx->GainAttn_fx, 27852 );
    }

    *GainFrame = Mult_32_16( st_fx->GainFrame_prevfrm_fx, st_fx->GainAttn_fx ); /* Q18 */
}

/*-------------------------------------------------------------------*
 * Dequant_lower_LSF()
 *
 * Dequantized the lower LSFs
 *-------------------------------------------------------------------*/


static void Dequant_lower_LSF_fx(
    const   Word16 lsf_idx[],               /* i  : LSF indices */
    Word16 lsf_q[]                  /* o  : Quantized LSFs */
    ,const Word16 rf_mode_SWB1K2
)
{
    Word16 i;

    IF(!rf_mode_SWB1K2)
    {
        lsf_q[0] = lsf_q_cb_fx[0][lsf_idx[0]];
        move16();
        FOR( i = 1; i < NUM_Q_LSF; i++ )
        {
            lsf_q[i] = add( lsf_q_cb_fx[i][lsf_idx[i]], lsf_q[i - 1] );
            move16();
        }
    }
    ELSE
    {
        lsf_q[0] = lsf_q_cb_rf_fx[0][lsf_idx[0]];
        move16();
        FOR( i = 1; i < NUM_Q_LSF; i++ )
        {
            lsf_q[i] = add( lsf_q_cb_rf_fx[i][lsf_idx[i]], lsf_q[i - 1] );
            move16();
        }
    }
}

/*-------------------------------------------------------------------*
 * Map_higher_LSF()
 *
 * Map the higher LSFs from the lower LSFs
 *-------------------------------------------------------------------*/

static void Map_higher_LSF_fx(
    Word16 lsf_q[],                 /* i/o : Quantized lower LSFs     */
    const   Word16 m,                       /* i   : Mirroring point       */
    const   Word16 grid_in[]                /* i   : Input LSF smoohthing grid */ )
{
    Word16 lsf_map[NUM_MAP_LSF];
    Word16 grid[NUM_MAP_LSF];
    Word16 last_q_lsf;
    Word16 lsf_smooth[NUM_MAP_LSF];
    Word16 offset;
    Word16 i;
    Word16 scale;

    FOR( i = 0; i < NUM_MAP_LSF; i++ )
    {
        lsf_map[i] = sub( shl( m, 1 ), lsf_q[NUM_MAP_LSF - 1 - i] );
        move16();
    }

    IF( sub( m, MAX_LSF_FX_BY_2 ) > 0 )
    {
        offset = lsf_map[0];
        move16();
        scale = div_s( sub( MAX_LSF_FX, m ), m );
        FOR( i = 0; i < NUM_MAP_LSF; i++ )
        {
            lsf_map[i] = add( mult_r( sub( lsf_map[i], offset ), scale ), offset );
            move16();
        }
    }

    last_q_lsf = lsf_q[NUM_Q_LSF - 1];
    move16();
    scale = sub( MAX_LSF_FX, last_q_lsf );

    FOR( i = 0; i < NUM_MAP_LSF; i++ )
    {
        grid[i] = add( mult_r( grid_in[i], scale ), last_q_lsf );
        move16();
    }

    FOR( i = 0; i < NUM_MAP_LSF; i++ )
    {
        lsf_smooth[i] = sub( mult_r( grid_smoothing_fx[i], grid[i] ),
                             mult_r( lsf_map[i], add( grid_smoothing_fx[i], FL2WORD16(-1.0f) ) ) );
        move16();
    }

    FOR( i = 0; i < NUM_MAP_LSF; i++ )
    {
        lsf_q[NUM_Q_LSF + i] = lsf_smooth[i];
        move16();
    }

    return;
}




static void Dequant_mirror_point_fx(
    const Word16 lsf_q[],               /* i/o : Quantized lower LSFs */
    const Word16 m_idx,                 /* i   : Mirror point index */ Word16* m /* i   : Mirroring point */ )
{
    *m = add( mirror_point_q_cb_fx[m_idx], lsf_q[NUM_Q_LSF - 1] );
    move16();

    return;
}


/*==========================================================================*/
/* FUNCTION      : static void dequantizeSHBparams_fx_9_1 ()        */
/*--------------------------------------------------------------------------*/
/* PURPOSE       : Dequantize super highband spectral envolope        */
/*          temporal gains and frame gain              */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                            */
/* _Word16 extl               i  : extension layer                */
/* _Word32 extl_brate         i  : extensiuon layer bitrate          */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                            */
/* _Word16 *Q_lsf,              o  : SHB LSF from de-quantization       Q15 */
/* _Word16 *Q_subgain,      o  : SHB subframe gains from de-quantization Q15*/
/* _Word32 *Q_framegrain      o  : SHB frame gain from de-quantization Q18  */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                          */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                            */
/*           _ None                          */
/*--------------------------------------------------------------------------*/
/* CALLED FROM :                               */
/*==========================================================================*/
static void dequantizeSHBparams_fx_9_1(
    Decoder_State_fx* st_fx,
    const Word16 extl,               /* i  : extension layer                 */
    Word32 extl_brate,         /* i  : extensiuon layer bitrate             */
    Word16* Q_lsf,             /* o  : SHB LSF from de-quantization        Q15*/
    Word16* Q_subgain,         /* o  : SHB subframe gains from de-quantization  Q15*/
    Word32* Q_framegrain,       /* o  : SHB frame gain from de-quantization      Q18*/
    Word16* uv_flag,            /* o  : unvoiced flag*/
    Word32* Q_shb_ener_sf,      /* o  : Q15                                         */
    Word16* Q_shb_res_gshape,   /* o  : Q14                                         */
    Word16* Q_mixFactors        /* o  : Q15                                         */
)
{
    Word16 i, j, idxLSF, idxSubGain, idxFrameGain;
    Word16 Q_combined_gains[NUM_SHB_SUBFR/4];
    Word16 lsf_q[LPC_SHB_ORDER];
    Word16 lsf_idx[NUM_Q_LSF];
    Word16 m_idx, grid_idx;
    Word16 m;
    Word32 L_tmp;
    Word16 tmp, frac, exp;
    Word16 idx_shb_fr_gain, idx_res_gs[5], idx_mixFac;
    Word16 temp_shb_ener_sf_fx;

    /* LSFs */

    IF( sub( extl, WB_TBE ) == 0 )
    {
        IF( L_sub( extl_brate, WB_TBE_0k35 ) == 0 )
        {
            IF( sub(st_fx->codec_mode,MODE2) == 0 )
            {
                idxFrameGain = st_fx->gFrame_WB_fx;
                idxLSF = st_fx->lsf_WB_fx;
            }
            ELSE
            {
                idxFrameGain = ( Word16 )get_next_indice_fx( st_fx, NUM_BITS_SHB_FrameGain_LBR_WB );
                idxLSF = ( Word16 )get_next_indice_fx( st_fx, NUM_BITS_LBR_WB_LSF );
            }

            Copy( lbr_wb_bwe_lsfvq_cbook_2bit_fx + idxLSF * LPC_SHB_ORDER_LBR_WB, Q_lsf, LPC_SHB_ORDER_LBR_WB );
            set16_fx( Q_subgain, RECIP_ROOT_EIGHT_FX, NUM_SHB_SUBFR / 2 );
            Copy32( SHBCB_FrameGain16_fx + idxFrameGain, Q_framegrain, 1 );
        }
        ELSE
        {
            *uv_flag = ( Word16 )get_next_indice_fx( st_fx, 1 );
            idxSubGain = ( Word16 )get_next_indice_fx(  st_fx, NUM_BITS_SHB_SUBGAINS );
            move16();
            idxFrameGain = ( Word16 )get_next_indice_fx( st_fx, NUM_BITS_SHB_FrameGain );
            move16();
            idxLSF = ( Word16 )get_next_indice_fx( st_fx, NUM_BITS_WB_LSF );
            move16();
            Copy( wb_bwe_lsfvq_cbook_8bit_fx + idxLSF * LPC_SHB_ORDER_WB, Q_lsf, LPC_SHB_ORDER_WB );
            Copy( HBCB_SubGain5bit_fx + idxSubGain * NUM_SHB_SUBFR / 4, Q_combined_gains, NUM_SHB_SUBFR / 4 );

            FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
            {

                L_tmp = L_mult( Q_combined_gains[i], 21771 );   /*Q26  0.166096 in Q17 */
                L_tmp = L_shr( L_tmp, 10 );
                frac = L_Extract_lc( L_tmp, &exp );
                tmp = extract_l( Pow2( 14, frac ) );            /* Put 14 as exponent so that */
                /* output of Pow2() will be: */
                /* 16384 < Pow2() <= 32767 */
                Q_combined_gains[i] = shl( tmp, add( exp, 1 ) ); /* Q15 */
            }

            FOR( i=0;  i < NUM_SHB_SUBFR / 2; i+= 2)
            {
                Q_subgain[i]     = Q_combined_gains[i / 2];
                move16();
                Q_subgain[i + 1] = Q_combined_gains[i / 2];
                move16();
            }

            /* frame gain */
            Copy32( SHBCB_FrameGain64_fx + idxFrameGain, Q_framegrain, 1 );
        }
    }
    ELSE /* SWB TBE DEC */
    {
        IF(sub(st_fx->codec_mode,MODE2) == 0)
        {
            idxSubGain = st_fx->idxSubGains_fx;
            idxFrameGain = st_fx->idxFrameGain_fx;
        }
        ELSE
        {
            idxSubGain = ( Word16 )get_next_indice_fx( st_fx, NUM_BITS_SHB_SUBGAINS );
            idxFrameGain = ( Word16 )get_next_indice_fx( st_fx, NUM_BITS_SHB_FRAMEGAIN );
        }

        test();
        IF( L_sub(st_fx->total_brate_fx, ACELP_24k40) == 0 || L_sub(st_fx->total_brate_fx, ACELP_32k) == 0 )
        {
            IF(sub(st_fx->codec_mode,MODE2) == 0)
            {
                idx_shb_fr_gain = st_fx->idx_shb_fr_gain_fx;
            }
            ELSE
            {
                idx_shb_fr_gain = get_next_indice_fx( st_fx, NUM_BITS_SHB_ENER_SF );
            }
            temp_shb_ener_sf_fx = usdequant_fx(idx_shb_fr_gain, 0, 86); /* 86 = 0.042f in Q11 = Qin-1 */
            /* o: temp_shb_ener_sf_fx in Q12 */

            /* *Q_shb_ener_sf = Pow(10.0, temp_shb_ener_sf_fx );        */
            /*                = pow(2, 3.321928*temp_shb_ener_sf_fx)    */
            L_tmp = L_mult(temp_shb_ener_sf_fx, 27213 );    /* 3.321928 in Q13 -> L_tmp in Q12+Q13+1 = Q26 */
            L_tmp = L_shl( L_tmp, -10 );                    /* bring L_tmp from Q26 to Q16 */
            frac = L_Extract_lc( L_tmp, &exp );             /* Extract exponent */
            L_tmp = Pow2(14, frac );
            *Q_shb_ener_sf = L_shl(L_tmp, exp-14+0 );      /* In Q_ener: 2*Q_shb+1, Q_shb = 0; */

            FOR(i=0; i<5; i++)
            {
                IF(sub(st_fx->codec_mode,MODE2) == 0)
                {
                    idx_res_gs[i] = st_fx->idx_res_gs_fx[i];
                    move16();
                }
                ELSE
                {
                    idx_res_gs[i] = get_next_indice_fx( st_fx, NUM_BITS_SHB_RES_GS );
                    move16();
                }
                Q_shb_res_gshape[i] = usdequant_fx(idx_res_gs[i],
                                                   FL2WORD16_SCALE(0.125f, 15-14), /*2048 = 0.125 in Q14 */
                                                   FL2WORD16_SCALE(0.125f, 15-13)  /*1024 = 0.125 in Q13 */
                                                  );
                move16();
                /* o: Q_shb_res_gshape in Q14 */
            }

            IF(sub(st_fx->codec_mode,MODE2) == 0)
            {
                idx_mixFac = st_fx->idx_mixFac_fx;
                move16();
            }
            ELSE
            {
                idx_mixFac = (Word16)get_next_indice_fx( st_fx, NUM_BITS_SHB_VF );
            }
            *Q_mixFactors = usdequant_fx(idx_mixFac, 4096 /* 0.125 in Q15 */, 2048 /* 0.125 in Q14 */);
            move16();
            /* o: Q_mixFactors in Q15 */
        }
        ELSE
        {
            *Q_shb_ener_sf = L_deposit_l(0);
            *Q_mixFactors = 0;
            move16();
            set16_fx(Q_shb_res_gshape, 0, 5);
        }

        /* LSFs */


        test();
        test();
        test();
        test();
        test();
        IF( (st_fx->rf_flag == 0) && !((L_sub(st_fx->total_brate_fx, ACELP_9k60) == 0) || ((st_fx->total_brate_fx == 0) && ( (L_sub(st_fx->last_total_brate_fx, ACELP_9k60) == 0) || (L_sub(st_fx->last_total_brate_fx, ACELP_13k20) == 0 && sub(st_fx->rf_flag_last, 1) == 0) ))) )

        {
            /* LSFs */
            test();
            test();
            test();
            IF ( L_sub(extl_brate, SWB_TBE_1k6) == 0 || L_sub(extl_brate, FB_TBE_1k8) == 0 || L_sub(extl_brate, SWB_TBE_2k8) == 0  || L_sub(extl_brate, FB_TBE_3k0) == 0 )
            {
                IF(sub(st_fx->codec_mode,MODE2) == 0)
                {
                    FOR (i = 0; i < NUM_Q_LSF; i++)
                    {
                        lsf_idx[i] = st_fx->lsf_idx_fx[i];
                        move16();

                    }
                }
                ELSE
                {
                    FOR (i = 0; i < NUM_Q_LSF; i++)
                    {
                        lsf_idx[i] = (Word16)get_next_indice_fx(st_fx, lsf_q_num_bits[i]);
                        move16();
                    }
                }
            }
            ELSE IF( extl_brate == SWB_TBE_1k2 )
            {
                FOR (i = 0; i < NUM_Q_LSF; i++)
                {
                    lsf_idx[i] = (Word16)get_next_indice_fx(st_fx, lsf_q_num_bits_rf[i]);
                }
            }

            Dequant_lower_LSF_fx(lsf_idx, lsf_q,(extl_brate == SWB_TBE_1k2));

            IF(sub(st_fx->codec_mode,MODE2) == 0)
            {
                m_idx = st_fx->m_idx_fx;
            }
            ELSE
            {
                m_idx = (Word16)get_next_indice_fx( st_fx,  MIRROR_POINT_BITS );
                move16();
            }

            Dequant_mirror_point_fx( lsf_q, m_idx, &m );

            IF(sub(st_fx->codec_mode,MODE2) == 0)
            {
                grid_idx = st_fx->grid_idx_fx;
            }
            ELSE
            {
                grid_idx = (Word16)get_next_indice_fx( st_fx, NUM_LSF_GRID_BITS );
                move16();
            }

            Map_higher_LSF_fx( lsf_q, m, lsf_grid_fx[grid_idx] );

            FOR (i = 0; i < LPC_SHB_ORDER; i++)
            {
                Q_lsf[i] = sub( 16384, lsf_q[LPC_SHB_ORDER - 1 - i] );
                move16();
            }
        }
        ELSE
        {
            set16_fx(lsf_idx, 0, 5);
            Copy(st_fx->lsf_idx_fx, lsf_idx, 5);
            grid_idx = 0;
            m_idx = 0;
            Copy(swb_tbe_lsfvq_cbook_8b + lsf_idx[0]*LPC_SHB_ORDER, Q_lsf, LPC_SHB_ORDER);
        }

        space_lsfs_fx( Q_lsf, LPC_SHB_ORDER );

        /* Dequantize subgain indices */
        j = idxSubGain * NUM_SHB_SUBGAINS;
        move16();
        FOR( i = 0; i < NUM_SHB_SUBGAINS; i++ )
        {
            /*    Q_subgain[i] = (float) pow(10.0, SHBCB_SubGain5bit[j++]); */

            L_tmp = L_mult( SHBCB_SubGain5bit_fx[j++], 27213 ); /*Q28  3.321928 in Q13 */
            L_tmp = L_shr( L_tmp, 12 );
            frac = L_Extract_lc( L_tmp, &exp );
            tmp = extract_l( Pow2( 14, frac ) );        /* Put 14 as exponent so that */
            /* output of Pow2() will be: */
            /* 16384 < Pow2() <= 32767 */
            Q_subgain[i] = shl( tmp, add( exp, 1 ) ); /*Q15*/ move16();
        }

        FOR( i = NUM_SHB_SUBFR - 1; i >= 0; i-- )
        {
            Q_subgain[i] = Q_subgain[i * NUM_SHB_SUBGAINS / NUM_SHB_SUBFR];
            move16();
        }

        /* Frame gain */
        *Q_framegrain = L_mac( SHB_GAIN_QLOW_FX, idxFrameGain, SHB_GAIN_QDELTA_FX );
        move32();/*Q18*/
        L_tmp = Mult_32_16( *Q_framegrain, 27213 ); /*Q16*/   /* 3.321928 in Q13 */
        frac = L_Extract_lc( L_tmp, &exp );
        L_tmp = Pow2( 30, frac );
        *Q_framegrain = L_shl( L_tmp, sub( exp, 12 ) ); /*Q18*/
    }

    return;
}

/*-------------------------------------------------------------------*
 * fb_tbe_dec()
 *
 * FB TBE decoder, 14(resp. 15.5) - 20 kHz band decoding module
 *-------------------------------------------------------------------*/
void fb_tbe_dec_fx(
    Decoder_State_fx *st,                /* i/o: encoder state structure                 */
    const Word16  fb_exc[],           /* i  : FB excitation from the SWB part         */
    Word16  Q_fb_exc,
    Word16 *hb_synth,           /* o  : high-band synthesis                     */
    Word16  hb_synth_exp
)

{
    Word16 i;
    Word16 ratio = 0;
    Word32 fb_exc_energy = 0;
    Word16 fb_synth[L_FRAME48k];

    /* decode FB slope information */
    test();
    test();
    IF ( sub(st->extl_fx,FB_TBE) == 0 && !st->bfi_fx )
    {
        IF( sub(st->codec_mode,MODE2) == 0 )
        {
            i = st->idxGain_fx;
            move16();
        }
        ELSE
        {
            i = (Word16)get_next_indice_fx( st, 4 );
        }
        ratio = shl(1,i);
    }
    ELSE if ( sub(st->extl_fx,FB_TBE) == 0 && st->bfi_fx )
    {
        ratio = st->prev_fbbwe_ratio_fx;
        move16();
    }

    fb_exc_energy = sum2_fx(fb_exc,L_FRAME16k);

    /* FB TBE synthesis */
    synthesise_fb_high_band_fx( fb_exc,Q_fb_exc,fb_synth,fb_exc_energy,ratio, st->L_frame_fx, st->bfi_fx, &(st->prev_fbbwe_ratio_fx), st->fbbwe_hpf_mem_fx, hb_synth_exp);

    /* add the fb_synth component to the hb_synth component */
    /*  v_add_fx( hb_synth, fb_synth, hb_synth, L_FRAME48k );*/
    FOR (i=0; i<L_FRAME48k; i++)
    {
        hb_synth[i] = add(hb_synth[i],fb_synth[i]);
        move16();
    }
    return;
}

void tbe_read_bitstream_fx(
    Decoder_State_fx *st_fx
)
{
    Word16 i;

    test();
    test();
    test();
    test();
    test();
    IF ( (sub(st_fx->rf_flag,1)==0 || L_sub(st_fx->total_brate_fx,ACELP_9k60) == 0 ) && sub(st_fx->bwidth_fx,WB) == 0)
    {
        /* WB LSF */
        st_fx->lsf_WB_fx = get_next_indice_fx(st_fx, NUM_BITS_LBR_WB_LSF);

        /* WB frame gain */
        st_fx->gFrame_WB_fx = get_next_indice_fx(st_fx, NUM_BITS_SHB_FrameGain_LBR_WB);
    }
    ELSE IF ( ( L_sub( st_fx->total_brate_fx, ACELP_9k60 ) >= 0 ) && ( L_sub( st_fx->total_brate_fx, ACELP_32k ) <= 0 ) &&
              ( ( sub( st_fx->bwidth_fx, SWB ) == 0 ) || ( sub( st_fx->bwidth_fx, FB ) == 0 ) ) )
    {
        test();
        IF( (st_fx->rf_flag == 0) && (L_sub(st_fx->total_brate_fx, ACELP_9k60) > 0) )
        {
            FOR (i = 0; i < NUM_Q_LSF; i++)
            {
                st_fx->lsf_idx_fx[i] = get_next_indice_fx(st_fx, lsf_q_num_bits[i]);
                move16();
            }

            st_fx->m_idx_fx = get_next_indice_fx(st_fx, MIRROR_POINT_BITS);

            st_fx->grid_idx_fx = get_next_indice_fx(st_fx, NUM_LSF_GRID_BITS);

        }
        ELSE
        {
            st_fx->lsf_idx_fx[0] = get_next_indice_fx(st_fx, 8);
            move16();
            st_fx->m_idx_fx = 0;
            move16();
            st_fx->grid_idx_fx = 0;
            move16();
        }

        /* shape gains */
        st_fx->idxSubGains_fx = get_next_indice_fx(st_fx, NUM_BITS_SHB_SUBGAINS);

        /* frame gain */
        st_fx->idxFrameGain_fx = get_next_indice_fx(st_fx, NUM_BITS_SHB_FRAMEGAIN);

        IF ( L_sub( st_fx->total_brate_fx, ACELP_24k40 ) >= 0 )
        {
            /* sub frame energy*/
            st_fx->idx_shb_fr_gain_fx = get_next_indice_fx(st_fx, NUM_BITS_SHB_ENER_SF);

            /* gain shapes residual */
            FOR (i = 0; i < NB_SUBFR16k; i++)
            {
                st_fx->idx_res_gs_fx[i] = get_next_indice_fx(st_fx, NUM_BITS_SHB_RES_GS);
                move16();
            }

            /* voicing factor */
            st_fx->idx_mixFac_fx = get_next_indice_fx(st_fx, NUM_BITS_SHB_VF);
        }

        IF (sub(st_fx->tec_tfa, 1) == 0)
        {
            st_fx->tec_flag = get_next_indice_fx(st_fx, BITS_TEC);
            st_fx->tfa_flag = get_next_indice_fx(st_fx, BITS_TFA);
            test();
            IF (st_fx->tfa_flag && st_fx->tec_flag)
            {
                st_fx->tec_flag = 2;
                move16();
                st_fx->tfa_flag = 0;
                move16();
            }
        }
        ELSE
        {
            st_fx->tec_flag = 0;
            move16();
            st_fx->tfa_flag = 0;
            move16();
        }
    }

    IF ( sub( st_fx->bwidth_fx, FB ) == 0 )
    {
        st_fx->idxGain_fx = get_next_indice_fx(st_fx, 4);
    }
}


/*---------------------------------------------------------------------*
* GenTransition()
*
* Generate a highband transition signal from the gain shape overlap
* buffer to fill the gap caused by the delay alignment buffer when
* switching from TBE to IGF
*---------------------------------------------------------------------*/
void GenTransition_fx(
    const Word16 *input,                     /* i  : gain shape overlap buffer              */
    const Word16 *old_hb_synth,              /* i  : synthesized HB from previous frame     */
    Word16 length,                     /* i  : targeted length of transition signal   */
    Word16 *output,                    /* o  : synthesized transitions signal         */
    Word32 Hilbert_Mem[],              /* i/o: memory                                 */
    Word16 state_lsyn_filt_shb_local[],/* i/o: memory                                 */
    Word16 *syn_dm_phase,
    Word32   target_fs,
    Word16 *up_mem,
    Word16   rf_flag
    , Word32 bitrate
)
{
    Word16 i;
    Word16 syn_overlap_32k[L_FRAME32k];
    Word32 L_tmp;
    Word16 ol_len = 2*SHB_OVERLAP_LEN;

    /* upsample overlap snippet */
    Interpolate_allpass_steep_fx( input, state_lsyn_filt_shb_local, SHB_OVERLAP_LEN, syn_overlap_32k );

    /* perform spectral flip and downmix with overlap snippet to match HB synth  */
    test();
    IF( (rf_flag != 0) || L_sub( bitrate, ACELP_9k60 ) == 0 )
    {
        flip_and_downmix_generic_fx( syn_overlap_32k, syn_overlap_32k, 2*SHB_OVERLAP_LEN, Hilbert_Mem,
                                     Hilbert_Mem + HILBERT_ORDER1, Hilbert_Mem + (HILBERT_ORDER1+2*HILBERT_ORDER2),
                                     syn_dm_phase );
    }
    ELSE
    {
        FOR(i = 0; i < ol_len; i=i+2)
        {
            syn_overlap_32k[i] = negate(syn_overlap_32k[i]);
            move16();
            syn_overlap_32k[i+1] = syn_overlap_32k[i+1];
            move16();
        }
    }

    /* cross fade of overlap snippet and mirrored HB synth from previous frame */
    FOR ( i = 0; i < ol_len; i++ )
    {
        L_tmp = L_mult0( window_shb_32k_fx[i], old_hb_synth[L_FRAME32k-1-i] );
        output[i] = round_fx( L_mac0( L_tmp, window_shb_32k_fx[2*L_SHB_LAHEAD-1-i], syn_overlap_32k[i] ) );
    }

    /* fill transition signal with mirrored HB synth from previous frame to fully fill delay alignment buffer gap */
    FOR ( ; i < length; i++)
    {
        output[i] = old_hb_synth[L_FRAME32k-1-i];
    }

    IF ( L_sub( target_fs, 48000 ) == 0 )
    {
        interpolate_3_over_2_allpass_fx( output, length, output, up_mem, allpass_poles_3_ov_2 );
    }

    return;
}



void TBEreset_dec_fx(
    Decoder_State_fx *st_fx,          /* i/o: decoder state structure                 */
    Word16 bandwidth                  /* i  : bandwidth mode                          */
)
{
    set16_fx( st_fx->old_bwe_exc_fx, 0, PIT16k_MAX * 2 );
    st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);
    st_fx->prev_Q_bwe_exc = 31;
    move16();

    test();
    IF( sub(bandwidth, WB) == 0 )
    {
        wb_tbe_extras_reset_fx( st_fx->mem_genSHBexc_filt_down_wb2_fx, st_fx->mem_genSHBexc_filt_down_wb3_fx );
        wb_tbe_extras_reset_synth_fx( st_fx->state_lsyn_filt_shb_fx, st_fx->state_lsyn_filt_dwn_shb_fx, st_fx->state_32and48k_WB_upsample_fx
                                      ,st_fx->mem_resamp_HB_fx
                                    );

        set16_fx( st_fx->state_syn_shbexc_fx, 0, L_SHB_LAHEAD/4 );
        set16_fx( st_fx->syn_overlap_fx, 0, L_SHB_LAHEAD );
        set32_fx( st_fx->mem_csfilt_fx, 0, 2 );
    }
    ELSE IF( sub(bandwidth, SWB) == 0 || sub(bandwidth, FB) == 0 )
    {
        swb_tbe_reset_fx( st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                          st_fx->syn_overlap_fx, st_fx->state_syn_shbexc_fx, &(st_fx->tbe_demph_fx),
                          &(st_fx->tbe_premph_fx), st_fx->mem_stp_swb_fx, &(st_fx->gain_prec_swb_fx) );

        swb_tbe_reset_synth_fx( st_fx->genSHBsynth_Hilbert_Mem_fx, st_fx->genSHBsynth_state_lsyn_filt_shb_local_fx );

        set16_fx( st_fx->GainShape_Delay, 0, NUM_SHB_SUBFR/2 );
        set16_fx( st_fx->int_3_over_2_tbemem_dec_fx, 0, INTERP_3_2_MEM_LEN);

        IF( sub(bandwidth, FB) == 0 )
        {
            st_fx->prev_fb_ener_adjust_fx = 0;
            fb_tbe_reset_synth_fx( st_fx->fbbwe_hpf_mem_fx, &st_fx->prev_fbbwe_ratio_fx );
        }
    }

    return;
}
