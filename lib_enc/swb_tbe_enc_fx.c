/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>
#include "options.h"
#include "cnst_fx.h"
#include "prot_fx.h"
#include "rom_com_fx.h"
#include "stl.h"

/*-----------------------------------------------------------------*
* Local functions
*-----------------------------------------------------------------*/
static void return_M_Least_fx_GainFrame( const Word32* inp,
        const Word32* codebook, const Word16 num_grp,
        const Word16 interNum, Word16* least );


static void singlevectortest_gain_fx( const Word32* inp, const Word16 dimen,
                                      const Word16 cb_size, Word16* index,
                                      Word32* recon, const Word32* codebook );


static void determine_gain_weights_fx( const Word32* gain, Word16* weights,
                                       const Word16 dims );

static void QuantizeSHBsubgains_fx( Encoder_State_fx* st_fx, Word16* subgains, const Word16 extl );


static void QuantizeSHBframegain_fx( Encoder_State_fx* st_fx, Word32* GainFrame, const Word16 extl, Word32 extl_brate
                                     ,Word16 *rf_gainFrame_ind);

static Word16 closest_centroid_fx( const Word16* data, const Word16* weights,
                                   const Word16* quantizer, const Word16 centroids, const Word16 length );

static Word16 closest_centroid_lc_fx( const Word16* data, const Word16* quantizer, const Word16 centroids );

static void EstimateSHBFrameGain_fx( const Word16 length,
                                     const Word16* oriSHB, const Word16 Q_oriSHB,
                                     const Word16* synSHB, const Word16 Q_synSHB, Word16* subgain,
                                     Word32* GainFrame, const Word16* win_shb, const Word16* subwin_shb );



static void EstimateSHBGainShape_fx( const Word16 length,
                                     const Word16* oriSHB, const Word16 Q_oriSHB,
                                     const Word16* synSHB, const Word16 Q_synSHB,
                                     Word16* subgain, const Word16* subwin );


static Word32 pow_off_pk_fx( Word16 a[], Word16 len, Word16 step );

static void find_max_mem_enc( Encoder_State_fx *st_fx, Word16 *n_mem, Word16 *n_mem2 );
static void rescale_genSHB_mem_enc( Encoder_State_fx* st_fx, Word16 sf );
static void find_max_mem_wb_enc( Encoder_State_fx* st_fx, Word16* n_mem );
static void rescale_genWB_mem_enc( Encoder_State_fx* st_fx, Word16 sf );

static void Quant_lower_LSF_fx( const Word16 lsf[],
                                Word16 lsf_q[],
                                Word16 lsf_idx[]);

static Word16 Quant_mirror_point_fx( const Word16 lsf[], const Word16 lsf_q[],
                                     Word16* m );
static Word16 Find_LSF_grid_fx( const Word16 lsf[], Word16 lsf_q[],
                                const Word16 m );
static void Quant_BWE_LSF_fx( Encoder_State_fx* st_fx,  const Word16 lsp_shb[], Word16 Q_lsfs[]);
static void Quant_shb_ener_sf_fx(Encoder_State_fx *st_fx, Word32 *shb_ener_sf_fx_32, Word16 Q_shb);
static void Quant_shb_res_gshape_fx(Encoder_State_fx *st_fx, Word16 *shb_res_gshape_fx);

static void gainFrSmooth_En_fx(Encoder_State_fx *st_fx,
                               Word16 *shb_frame_fx,
                               const Word16 *lpc_shb_fx,
                               const Word16 *lsp_shb_fx,
                               Word16 *MA_lsp_shb_spacing,
                               Word16 *frGainAttenuate,
                               Word16 *frGainSmoothEn
                              );


/*-------------------------------------------------------------------*
 * find_max_mem_enc()
 *
 * Find norm and max in TBE memories and past buffers
 *-------------------------------------------------------------------*/
void find_max_mem_enc(
    Encoder_State_fx *st_fx,
    Word16 *n_mem,
    Word16 *n_mem2
)
{
    Word16 i;
    Word16 n_mem_32;
    Word16 max = 0;
    Word32 Lmax = 0;
    Word16 tempQ15, max2 = 0;

    /* old BWE exc max */
    FOR( i = 0; i < NL_BUFF_OFFSET; i++ )
    {
        tempQ15 = abs_s( st_fx->old_bwe_exc_extended_fx[i] );
        max = s_max( max, tempQ15 );
    }

    /* decimate all-pass steep memory */
    FOR ( i = 0; i < (2*ALLPASSSECTIONS_STEEP+1); i++ )
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
}


/*-------------------------------------------------------------------*
 * rescale_genSHB_mem_enc()
 *
 * Rescale genSHB memories
 *-------------------------------------------------------------------*/
void rescale_genSHB_mem_enc( Encoder_State_fx* st_fx, Word16 sf )
{
    Word16 i;

    FOR( i = 0; i <NL_BUFF_OFFSET; i++ )
    {
        st_fx->old_bwe_exc_extended_fx[i] = shl( st_fx->old_bwe_exc_extended_fx[i], sf );
        move16();
    }

    FOR ( i = 0; i < 7; i++ )
    {
        st_fx->mem_genSHBexc_filt_down_shb_fx[i] =  shl( st_fx->mem_genSHBexc_filt_down_shb_fx[i], sf );
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

void find_max_mem_wb_enc( Encoder_State_fx* st_fx, Word16* n_mem )
{
    Word16 i;
    Word16 n_mem_32;
    Word16 max =0;
    Word32 Lmax =0;

    FOR ( i = 0; i < NL_BUFF_OFFSET; i++ )
    max = s_max( max, abs_s( st_fx->old_bwe_exc_extended_fx[i] ) );

    FOR ( i = 0; i < 7; i++ )
    {
        IF ( abs_s( st_fx->mem_genSHBexc_filt_down_shb_fx[i] ) > max )
        max = abs_s( st_fx->mem_genSHBexc_filt_down_shb_fx[i] );
    }

    FOR ( i = 0; i < 7; i++ )
    {
        IF ( abs_s( st_fx->mem_genSHBexc_filt_down_wb2_fx[i] ) > max )
        max = abs_s( st_fx->mem_genSHBexc_filt_down_wb2_fx[i] );
    }

    FOR ( i = 0; i < 7; i++ )
    {
        IF ( abs_s( st_fx->mem_genSHBexc_filt_down_wb3_fx[i] ) > max )
        max = abs_s( st_fx->mem_genSHBexc_filt_down_wb3_fx[i] );
    }

    FOR ( i = 0; i < 10; i++ )
    {
        IF ( abs_s( st_fx->state_lpc_syn_fx[i] ) > max )
        max = abs_s( st_fx->state_lpc_syn_fx[i] );
    }

    FOR ( i = 0; i < 5; i++ )
    {
        IF ( abs_s( st_fx->state_syn_shbexc_fx[i] ) > max )
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

    FOR ( i = 0; i < 2; i++ )
    {
        IF ( L_abs( st_fx->mem_csfilt_fx[i] ) > Lmax )
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
    move16();
    *n_mem = s_max( *n_mem, 0 );
}

void rescale_genWB_mem_enc( Encoder_State_fx* st_fx, Word16 sf )
{
    Word16 i;

    FOR ( i = 0; i < NL_BUFF_OFFSET; i++ )
    {
        st_fx->old_bwe_exc_extended_fx[i] = shl( st_fx->old_bwe_exc_extended_fx[i], sf );
        move16();
    }

    FOR ( i = 0; i < 10; i++ )
    {
        st_fx->state_lpc_syn_fx[i] = shl( st_fx->state_lpc_syn_fx[i], sf );
        move16();
    }

    FOR ( i = 0; i < 5; i++ )
    {
        st_fx->state_syn_shbexc_fx[i] = shl( st_fx->state_syn_shbexc_fx[i], sf );
        move16();
    }

    FOR ( i = 0; i < 7; i++ )
    {
        st_fx->mem_genSHBexc_filt_down_shb_fx[i] = shl( st_fx->mem_genSHBexc_filt_down_shb_fx[i], sf );
        move16();
    }

    FOR ( i = 0; i < 7; i++ )
    {
        st_fx->mem_genSHBexc_filt_down_wb2_fx[i] = shl( st_fx->mem_genSHBexc_filt_down_wb2_fx[i], sf );
        move16();
    }

    FOR ( i = 0; i < 7; i++ )
    {
        st_fx->mem_genSHBexc_filt_down_wb3_fx[i] = shl( st_fx->mem_genSHBexc_filt_down_wb3_fx[i], sf );
        move16();
    }

    FOR ( i = 0; i < 2; i++ )
    {
        st_fx->mem_csfilt_fx[i] = L_shl( st_fx->mem_csfilt_fx[i], sf );
        move32();
    }
}


/*-------------------------------------------------------------------*
 * InitSWBencBuffer()
 *
 * Initialize SWB buffers
 *-------------------------------------------------------------------*/
void InitSWBencBuffer_fx(
    Encoder_State_fx* st_fx /* i/o: SHB encoder structure */
)
{
    set16_fx( st_fx->old_speech_shb_fx, 0, L_LOOK_16k + L_SUBFR16k );
    set16_fx( st_fx->old_bwe_exc_fx, 0, ( PIT16k_MAX * 2 ) );
    st_fx->bwe_seed_fx[0] = 23;
    move16();
    st_fx->bwe_seed_fx[1] = 59;
    move16();
    set16_fx( st_fx->old_bwe_exc_extended_fx, 0, NL_BUFF_OFFSET );

    st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);

    set16_fx(st_fx->state_ana_filt_shb_fx,  0, (2*ALLPASSSECTIONS_STEEP+1));
    st_fx->prev_energy_fbe_fb_fx = 0;

    set32_fx( st_fx->elliptic_bpf_2_48k_mem_fx[0], 0, 4 );
    set32_fx( st_fx->elliptic_bpf_2_48k_mem_fx[1], 0, 4 );
    set32_fx( st_fx->elliptic_bpf_2_48k_mem_fx[2], 0, 4 );
    set32_fx( st_fx->elliptic_bpf_2_48k_mem_fx[3], 0, 4 );
    st_fx->prev_fb_energy_fx = 0;
    move16();

    set16_fx( st_fx->prev_lsp_shb_fx, 0, 10 );
    st_fx->prev_Q_bwe_exc = 31;
    move16();
    st_fx->prev_Q_bwe_syn = 31;
    move16();
    set16_fx( st_fx->prev_lsp_wb_fx, 0, 6 );
    set16_fx( st_fx->prev_lsp_wb_temp_fx, 0, 6 );

    set16_fx( st_fx->prev_lpc_wb_fx, 0, LPC_SHB_ORDER_WB);

    return;
}

/*-------------------------------------------------------------------*
 * ResetSHBbuffer_Enc()
 *
 *-------------------------------------------------------------------*/
void ResetSHBbuffer_Enc_fx(
    Encoder_State_fx* st_fx /* i/o: SHB encoder structure */
)
{
    /* states for the filters used in generating SHB excitation from WB excitation*/
    set16_fx( st_fx->mem_genSHBexc_filt_down_shb_fx, 0, 2*ALLPASSSECTIONS_STEEP+1);
    set32_fx( st_fx->mem_csfilt_fx, 0, 2 );

    /* states for the filters used in generating SHB signal from SHB excitation*/
    set16_fx( st_fx->state_syn_shbexc_fx, 0, L_SHB_LAHEAD );
    set16_fx( st_fx->state_lpc_syn_fx, 0, LPC_SHB_ORDER );


    /* states for the filters used in generating WB signal from WB excitation*/
    set16_fx( st_fx->decim_state1_fx, 0, 2*ALLPASSSECTIONS_STEEP+1 );
    set16_fx( st_fx->decim_state2_fx, 0, 2*ALLPASSSECTIONS_STEEP+1);
    set16_fx( st_fx->mem_genSHBexc_filt_down_wb2_fx, 0, 2*ALLPASSSECTIONS_STEEP+1 );
    set16_fx( st_fx->mem_genSHBexc_filt_down_wb3_fx, 0, 2*ALLPASSSECTIONS_STEEP+1 );

    /* overlap buffer used to Adjust SHB Frame Gain */
    set16_fx(st_fx->mem_stp_swb_fx, 0, LPC_SHB_ORDER);
    st_fx->gain_prec_swb_fx = 16384;/*Q14=1 */
    set16_fx( st_fx->syn_overlap_fx, 0, L_SHB_LAHEAD );
    st_fx->tbe_demph_fx = 0;
    st_fx->tbe_premph_fx = 0;


    return;
}


/*==========================================================================*/
/* FUNCTION : void wb_tbe_enc_fx() */
/*--------------------------------------------------------------------------*/
/* PURPOSE : WB TBE encoder, 6 - 8 kHz band encoding module */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* Word16 coder_type i : coding type */
/* Word16 *new_speech i : original input signal Q0 */
/* Word32 *bwe_exc_extended i : bandwidth extended exciatation 2*Q_new*/
/* Word16 voice_factors[] i : voicing factors Q15 */
/* Word16 pitch_buf[] i : pitch for each subframe Q6 */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* Word16 *synth o : WB SHB final synthesis */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* Encoder_State_fx *st_fx i/o: encoder state structure */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------*/
/* */
/*==========================================================================*/

#define WBTBE_LOOK_LSUFBR_5_OVER_16     (L_LOOK_12k8 + L_SUBFR)*5/16
#define WBTBE_LSUBFR_5_OVER_16          L_SUBFR*5/16
#define WBTBE_ANA_ALIGNDELAY           -(L_SHB_LAHEAD/4 + 5)
#define LFRAME16K_OVER_4                L_FRAME16k/4
#define WBTBE_LPCWIN_LENGTH             (L_LOOK_12k8 + L_SUBFR + L_FRAME) * 5/16 - 1

void wb_tbe_enc_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure             */
    const Word16 coder_type,           /* i  : coding type                         */
    const Word16 *hb_speech,           /* i  : HB target signal (6-8kHz) at 16kHz at Q-1 */
    const Word32 *bwe_exc_extended,    /* i  : bandwidth extended exciatation      */
    const Word16 Q_new,                /* i  : input HB speech Q factor */
    const Word16 voice_factors[],      /* i  : voicing factors                     */
    const Word16 pitch_buf[],          /* i  : pitch for each subframe             */
    const Word16 voicing_fx[]             /* i  : OL maximum normalized correlation   */
)
{
    Word16 i, j, k;
    Word16 hb_old_speech [(L_LOOK_12k8 + L_SUBFR + L_FRAME)*5/16];
    Word16 bwe_exc_extended_16[ L_FRAME32k+NL_BUFF_OFFSET ];

    Word16 shaped_wb_excitation[(L_FRAME16k + L_SHB_LAHEAD)/4];
    Word16 exc4kWhtnd[L_FRAME16k/4];
    /*Word16 ana_align_delay = WBTBE_ANA_ALIGNDELAY;  */ /* -L_SHB_LAHEAD/4 - 5 */
    Word32 GainFrame;
    Word16 GainShape[NUM_SHB_SUBFR];
    Word16 lpc_wb[LPC_SHB_ORDER_WB+1];
    Word32 lpc_wb_32_fx[LPC_SHB_ORDER_WB + 1];
    Word16 lsp_wb[LPC_SHB_ORDER_WB], weights_lsp[LPC_SHB_ORDER_WB] = {32767, 32767};
    Word16 *hb_new_speech, *hb_frame/*, hb_speech[ L_FRAME16k ]*/;
    Word16 R_h[ LPC_SHB_ORDER_WB + 2 ], R_l[ LPC_SHB_ORDER_WB + 2 ];
    Word16 Q_R;
    Word32 LepsP[LPC_SHB_ORDER_WB+1];

    Word32 prev_pow, curr_pow, Lscale;
    /* Word16 scale; */
    /*Word16 ramp_flag;*/
    Word32 p2m_in, p2m_out;
    /*Word16 cnt, max =0;*/
    Word16 n_mem, Q_bwe_exc, Q_bwe_exc_ext, exp_out; /* Q_hb_frame; */
    Word32 L_tmp, Lmax;
    Word16 tmp, exp, Q_out, sc;
    Word16 Q_ns = -1;

    Word16 pitBufAvg_fx, voicingBufAvg_fx;
    Word16 vf_modified_fx[NB_SUBFR16k];
    Word16 temp_wb_fac_fx, feedback_fx;
    Word16 lsp_spacing_fx;
    Word16 lsp_wb_temp_fx[ LPC_SHB_ORDER_WB ], lpc_wb_temp_fx[ LPC_SHB_ORDER_WB + 1 ];
    Word32 L_feedback;
    Word16 frac, exp1;
    Word16 uv_flag;
    Word16 dummy=0;
    Word16 avg_voice_fac;
    /*Word16 att = 32767;*/

    hb_new_speech = hb_old_speech + WBTBE_LOOK_LSUFBR_5_OVER_16;
    hb_frame = hb_old_speech + WBTBE_LSUBFR_5_OVER_16 + WBTBE_ANA_ALIGNDELAY;

    Copy( st_fx->old_speech_wb_fx, hb_old_speech, WBTBE_LOOK_LSUFBR_5_OVER_16 );
    Copy( hb_speech, hb_new_speech, LFRAME16K_OVER_4 );
    Copy( hb_old_speech + LFRAME16K_OVER_4, st_fx->old_speech_wb_fx, WBTBE_LOOK_LSUFBR_5_OVER_16 );

    test();
    test();
    test();
    test();
    test();
    test();
    IF ( ( sub (st_fx->last_extl_fx, WB_TBE) != 0 && sub( st_fx->last_extl_fx, WB_BWE) != 0 )
         && ( sub( st_fx->clas_fx, UNVOICED_CLAS) == 0  || ( sub( voicing_fx[0], 16384 ) < 0 && sub( voicing_fx[1], 16384 ) < 0 && sub( voicing_fx[2], 16384 ) < 0 ) )
         && st_fx->igf == 0 )
    {
        /* In case of unvoiced signals after switching cores, back-propagate the target signal */
        Copy( hb_speech, hb_old_speech, WBTBE_LOOK_LSUFBR_5_OVER_16 );

        i = WBTBE_LOOK_LSUFBR_5_OVER_16;
        move16();
        k = 0;
        move16();

        FOR ( j = 0; j < L_SUBFR16k; j = j + 4 )
        {
            L_tmp = L_mult( hb_old_speech[i], ola_win_shb_switch_fold_fx[j] );
            hb_old_speech[i] = mac_r( L_tmp, hb_speech[j], ola_win_shb_switch_fold_fx[ L_SUBFR16k - 4 - j ] );
            move16();
            i--;
            k++;
        }
    }

    autocorr_fx( hb_old_speech, LPC_SHB_ORDER_WB + 1, R_h, R_l, &Q_R, WBTBE_LPCWIN_LENGTH, win_lpc_hb_wb_fx, 0, 1 );

    E_LPC_lev_dur(R_h, R_l, lpc_wb_temp_fx, LepsP, LPC_SHB_ORDER_WB, NULL);

    Copy_Scale_sig( lpc_wb_temp_fx, lpc_wb_temp_fx, LPC_SHB_ORDER_WB+1, sub(norm_s(lpc_wb_temp_fx[0]),2) );

    /* convert into lsps and calculate weights */
    FOR ( i = 0; i <= LPC_SHB_ORDER_WB; i++ )
    {
        lpc_wb_32_fx[i] = L_negate( L_shr( L_deposit_h( lpc_wb_temp_fx[i] ), 1 ) );
        move32();
    }

    lpc2lsp_fx( &lpc_wb_32_fx[1], lsp_wb_temp_fx, st_fx->prev_lsp_wb_temp_fx, LPC_SHB_ORDER_WB );

    FOR ( i = 0; i < LPC_SHB_ORDER_WB; i++ )
    {
        st_fx->prev_lsp_wb_temp_fx[i] = lsp_wb_temp_fx[i];
        move16();
    }

    /* lsp_spacing_fx = 16384;                                             move16(); */
    lsp_spacing_fx = lsp_wb_temp_fx[0];
    move16();
    FOR ( i = 1; i < LPC_SHB_ORDER_WB; i++ )
    {
        /*if ( i == 0 )
        {
            tmp = lsp_wb_temp_fx[0]; move16();
        }
        else
        {*/
        tmp = sub( lsp_wb_temp_fx[i], lsp_wb_temp_fx[i - 1] );
        /*} */

        lsp_spacing_fx = s_min( lsp_spacing_fx, tmp );
    }

    /* Spectral smoothing of autocorrelation coefficients */
    FOR ( i = 1; i <= LPC_SHB_ORDER_WB; i++ )
    {
        L_tmp = Mpy_32( R_h[i], R_l[i], wac_h[i - 1], wac_l[i - 1] );
        L_Extract( L_tmp, &R_h[i], &R_l[i] );
    }
    R_l[0] = s_max( R_l[0], 1 );
    move16();

    test();
    IF ( sub(st_fx->rf_mode, 1) == 0 || L_sub(st_fx->extl_brate_fx, WB_TBE_0k35) == 0 )
    {
        E_LPC_lev_dur(R_h, R_l, lpc_wb, LepsP, LPC_SHB_ORDER_LBR_WB, NULL);
        Copy_Scale_sig( lpc_wb, lpc_wb, LPC_SHB_ORDER_LBR_WB+1, sub(norm_s(lpc_wb[0]),2) );

        /* Expand bandwidth of the LP coeffs */
        FOR ( i = 0; i <= LPC_SHB_ORDER_LBR_WB; i++ )
        {
            lpc_wb[i] = mult_r( lpc_wb[i], lpc_weights_fx[i] );
            move16();
        }

        /* convert into lsps and calculate weights */
        FOR ( i = 0; i <= LPC_SHB_ORDER_LBR_WB; i++ )
        {
            lpc_wb_32_fx[i] = L_negate( L_shr( L_deposit_h( lpc_wb[i] ),1 ) );
            move32();/*Q27 */
        }
        lpc2lsp_fx( &lpc_wb_32_fx[1], lsp_wb, st_fx->prev_lsp_wb_fx,LPC_SHB_ORDER_LBR_WB );

        FOR ( i = 0; i < LPC_SHB_ORDER_LBR_WB; i++ )
        {
            st_fx->prev_lsp_wb_fx[i] = lsp_wb[i];
            move16();
        }

        lsp_weights_fx( lsp_wb, weights_lsp, LPC_SHB_ORDER_LBR_WB, &Q_out );

        /* Quantization of LSFs */
        i = closest_centroid_fx( lsp_wb, weights_lsp, lbr_wb_bwe_lsfvq_cbook_2bit_fx, 4, LPC_SHB_ORDER_LBR_WB );
        IF( sub(st_fx->codec_mode, MODE2) == 0 )
        {
            st_fx->lsf_WB_fx = i;
            move16();
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_SHB_LSF, i, NUM_BITS_LBR_WB_LSF );
        }

        Copy( lbr_wb_bwe_lsfvq_cbook_2bit_fx + i * LPC_SHB_ORDER_LBR_WB,  lsp_wb, LPC_SHB_ORDER_LBR_WB );

        lsp2lpc_fx( &lpc_wb[1], lsp_wb, st_fx->prev_lpc_wb_fx, LPC_SHB_ORDER_LBR_WB );

        set16_fx( lpc_wb + LPC_SHB_ORDER_LBR_WB + 1, 0, ( LPC_SHB_ORDER_WB -  LPC_SHB_ORDER_LBR_WB ) );
        FOR ( i = 0; i < LPC_SHB_ORDER_WB; i++ )
        {
            st_fx->prev_lpc_wb_fx[i] = lpc_wb[i + 1];
            move16();
        }

        FOR ( i = 1; i < LPC_SHB_ORDER_LBR_WB + 1; i++ )
        {
            lpc_wb[i] = negate( lpc_wb[i] );
            move16();
        }
        lpc_wb[0] = 4096;
        move16();
    }
    ELSE /* 13.2kbps */
    {
        E_LPC_lev_dur(R_h, R_l, lpc_wb, LepsP, LPC_SHB_ORDER_WB, NULL);
        Copy_Scale_sig( lpc_wb, lpc_wb, LPC_SHB_ORDER_WB+1, sub(norm_s(lpc_wb[0]),2) );

        /* Expand bandwidth of the LP coeffs */
        FOR ( i = 0; i <= LPC_SHB_ORDER_WB; i++ )
        {
            lpc_wb[i] = mult_r( lpc_wb[i], lpc_weights_fx[i] );
            move16();
        }

        /* convert into lsps and calculate weights */
        FOR ( i = 0; i <= LPC_SHB_ORDER_WB; i++ )
        {
            lpc_wb_32_fx[i] = L_negate( L_shr( L_deposit_h( lpc_wb[i] ), 1 ) );
            move32();/*Q27 */
        }

        lpc2lsp_fx( &lpc_wb_32_fx[1], lsp_wb, st_fx->prev_lsp_wb_fx,  LPC_SHB_ORDER_WB );


        FOR( i = 0; i < LPC_SHB_ORDER_WB; i++ )
        {
            st_fx->prev_lsp_wb_fx[i] = lsp_wb[i];
            move16();
        }

        lsp_weights_fx( lsp_wb, weights_lsp, LPC_SHB_ORDER_WB, &Q_out );

        /* Quantization of LSFs */
        i = closest_centroid_fx( lsp_wb, weights_lsp, wb_bwe_lsfvq_cbook_8bit_fx, 256, LPC_SHB_ORDER_WB );/*move16(); */

        IF( sub(st_fx->codec_mode, MODE2) == 0 )
        {
            st_fx->lsf_WB_fx = i;
            move16();
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_SHB_LSF, i, NUM_BITS_WB_LSF );
        }
        Copy( wb_bwe_lsfvq_cbook_8bit_fx + i * LPC_SHB_ORDER_WB, lsp_wb, LPC_SHB_ORDER_WB );

        lsp2lpc_fx( &lpc_wb[1], lsp_wb, st_fx->prev_lpc_wb_fx,  LPC_SHB_ORDER_WB );


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

    uv_flag = 0;
    move16();
    test();
    if( L_sub(st_fx->extl_brate_fx, WB_TBE_1k05) == 0 && sub(st_fx->coder_type_raw_fx,UNVOICED) == 0 )
    {
        uv_flag = 1;
        move16();
    }

    Copy( voice_factors, vf_modified_fx, NB_SUBFR16k );
    IF( sub(coder_type,VOICED) == 0 )
    {
        FOR( i = 1; i < NB_SUBFR; i++ )
        {
            /*vf_modified[i] = 0.8f * voice_factors[i] + 0.2f * voice_factors[i-1];*/
            vf_modified_fx[i] = add( mult_r( 26214, voice_factors[i] ), mult_r( 6553, voice_factors[i - 1] ) );
        }
        IF( sub(st_fx->L_frame_fx, L_FRAME) != 0 )
        {
            vf_modified_fx[4] = add( mult_r( 26214, voice_factors[4] ), mult_r( 6553, voice_factors[3] ) );
        }
    }

    /* From low band excitation, generate highband excitation */
    Lmax = L_deposit_l(0);
    FOR( i = 0; i < L_FRAME32k; i++ )
    {
        Lmax = L_max( Lmax, L_abs( bwe_exc_extended[i] ) );
    }

    Q_bwe_exc = ( Lmax == 0 )?31:norm_l( Lmax );
    Q_bwe_exc = sub( Q_bwe_exc, 3 );
    Q_bwe_exc = add( Q_bwe_exc, add( Q_new, Q_new ) );

    find_max_mem_wb_enc( st_fx, &n_mem );

    if( sub(sub( Q_bwe_exc, st_fx->prev_Q_bwe_exc ),n_mem) > 0 )
    {
        Q_bwe_exc = add( st_fx->prev_Q_bwe_exc, n_mem );
    }

    IF( uv_flag )
    {
        if( sub( Q_bwe_exc, 20 ) > 0 )
        {
            Q_bwe_exc = 20;
            move16();/* restrict this to 20 due to the Q factor requireemnt of the random number generator (keep 1 bit headroom) */
        }
    }

    prev_pow = 0;
    move16();
    FOR( i = 0; i < L_SHB_LAHEAD / 4; i++ )
    {
        prev_pow = L_mac0( prev_pow, st_fx->state_syn_shbexc_fx[i], st_fx->state_syn_shbexc_fx[i] ); /* Q(2*st_fx->prev_Q_bwe_exc) */
    }

    rescale_genWB_mem_enc( st_fx, sub( Q_bwe_exc, st_fx->prev_Q_bwe_exc ) );

    Copy( st_fx->old_bwe_exc_extended_fx, bwe_exc_extended_16, NL_BUFF_OFFSET );
    sc = sub( Q_bwe_exc, add( Q_new, Q_new ) );
    FOR ( i = 0; i < L_FRAME32k; i++ )
    {
        bwe_exc_extended_16[i + NL_BUFF_OFFSET] = round_fx( L_shl( bwe_exc_extended[i], sc ) );
    }
    Copy( bwe_exc_extended_16 + L_FRAME32k, st_fx->old_bwe_exc_extended_fx, NL_BUFF_OFFSET );


    Copy( st_fx->state_syn_shbexc_fx, shaped_wb_excitation,  L_SHB_LAHEAD / 4 );
    Q_bwe_exc_ext = sub( Q_bwe_exc, 16 );


    GenShapedWBExcitation_fx( shaped_wb_excitation + L_SHB_LAHEAD / 4, lpc_wb, exc4kWhtnd, st_fx->mem_csfilt_fx,
                              st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->mem_genSHBexc_filt_down_wb2_fx, st_fx->mem_genSHBexc_filt_down_wb3_fx,
                              st_fx->state_lpc_syn_fx, coder_type, bwe_exc_extended_16, Q_bwe_exc_ext, st_fx->bwe_seed_fx,
                              vf_modified_fx,  uv_flag
                              , st_fx->igf
                            );

    curr_pow = 0;
    move16();
    FOR ( i = 0;  i < L_SHB_LAHEAD / 4; i++ )
    {
        curr_pow = L_mac0( curr_pow, shaped_wb_excitation[i + L_SHB_LAHEAD/4], shaped_wb_excitation[i + L_SHB_LAHEAD / 4] ); /* Q(2*Q_bwe_exc_ext) */
    }

    IF ( sub( voice_factors[0], 24576 ) > 0 )
    {
        curr_pow = L_shr( curr_pow, 2 ); /* Q(2*Q_bwe_exc_ext) */
    }

    Lscale = root_a_over_b_fx( curr_pow, shl_r( Q_bwe_exc_ext, 1 ), prev_pow, shl_r( sub( st_fx->prev_Q_bwe_exc, 16 ),  1 ), &exp );


    FOR ( i = 0; i < L_SHB_LAHEAD / 4 - 1; i++ )
    {
        L_tmp = Mult_32_16( Lscale, shaped_wb_excitation[i] ); /* Q(16-exp+Q_bwe_exc_ext) */
        shaped_wb_excitation[i] = round_fx( L_shl( L_tmp, exp ) ); /* Q_bwe_exc_ext */
    }

    Lscale = root_a_fx( Lscale, 31 - exp, &exp );
    L_tmp = Mult_32_16( Lscale, shaped_wb_excitation[L_SHB_LAHEAD / 4 - 1] ); /* Q(16-exp+Q_bwe_exc_ext) */
    shaped_wb_excitation[L_SHB_LAHEAD / 4 - 1] = round_fx( L_shl( L_tmp, exp ) ); /* Q_bwe_exc_ext */

    /* Update WB excitation */
    Copy( shaped_wb_excitation + L_FRAME16k / 4,  st_fx->state_syn_shbexc_fx, L_SHB_LAHEAD / 4 );


    EstimateSHBGainShape_fx( SHB_OVERLAP_LEN / 2, hb_frame, Q_ns,
                             shaped_wb_excitation, Q_bwe_exc_ext, GainShape,subwin_wb_fx );

    /* Gain frame adjustment factor */
    test();
    IF( GainShape[0] && st_fx->prev_wb_GainShape )
    {
        exp = norm_s( st_fx->prev_wb_GainShape );
        tmp = div_s( shl(1,sub( 14,exp )), st_fx->prev_wb_GainShape );
        L_tmp = L_mult( GainShape[0], tmp ); /* Q(30 - exp) */

        exp1 = norm_l( L_tmp );
        frac = Log2_norm_lc( L_shl( L_tmp, exp1 ) );
        exp1 = sub( exp, exp1 );
        L_tmp = Mpy_32_16( exp1, frac, 22713 );
        temp_wb_fac_fx = round_fx( L_shl( L_tmp, 10 ) );
    }
    ELSE
    {
        temp_wb_fac_fx = 0;
        move16();
    }
    L_feedback = L_mult0( temp_wb_fac_fx, temp_wb_fac_fx );
    FOR( i = 1; i < NUM_SHB_SUBFR / 4; i++ )
    {
        /* temp_swb_fac = (float)log( (GainShape[i]+0.00001f) / (GainShape[i-1]+0.0001f) ); */
        test();
        IF( GainShape[i] && GainShape[i - 1] )
        {
            exp = norm_s( GainShape[i - 1] );
            tmp = div_s( shl(1,sub( 14,exp )), GainShape[i - 1] );
            L_tmp = L_mult( GainShape[i], tmp );/*Q(30 - exp) */

            exp1 = norm_l( L_tmp );
            frac = Log2_norm_lc( L_shl( L_tmp, exp1 ) );
            move16();
            exp1 = sub( exp, exp1 );
            L_tmp = Mpy_32_16( exp1, frac, 22713 );
            temp_wb_fac_fx = round_fx( L_shl( L_tmp, 10 ) );
        }
        ELSE
        {
            temp_wb_fac_fx = 0;
            move16();
        }

        L_feedback = L_mac( L_feedback, temp_wb_fac_fx, temp_wb_fac_fx );

    }
    L_tmp = L_add( L_shr( L_feedback, 1 ), 1 << 21 ); /* Q30 */

    IF( L_tmp != 0 )
    {
        exp = norm_l( L_tmp );
        tmp = extract_h( L_shl( L_tmp, exp ) );
        exp = sub( sub( 30, exp ), 21 );
        tmp = div_s( 16384, tmp ); /* Q(15+exp)         */
        L_tmp = L_shr( L_mult( 13107, tmp ), exp ); /* Q31 */
        feedback_fx = round_fx( L_tmp ); /* Q15 */
    }
    ELSE
    {
        feedback_fx = 8738;
        move16();/* Q15 */
    }


    temp_wb_fac_fx = st_fx->prev_wb_GainShape;
    move16();
    FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
    {
        GainShape[i] = add( mult_r( sub( 32767, feedback_fx ), GainShape[i] ), mult_r( feedback_fx, temp_wb_fac_fx ) );
        move16();
        temp_wb_fac_fx = GainShape[i];
        move16();
    }

    st_fx->prev_wb_GainShape = GainShape[sub(shr(NUM_SHB_SUBFR,2),1)];
    move16();
    p2m_in = pow_off_pk_fx( GainShape, shr(NUM_SHB_SUBFR,2), 1 );
    move16();

    IF( L_sub(st_fx->extl_brate_fx,WB_TBE_0k35) == 0 )
    {
        FOR( i = 0; i < 8; i++ )
        {
            GainShape[i] = RECIP_ROOT_EIGHT_FX;
            move16();
        }

    }
    ELSE
    {
        push_indice_fx( st_fx, IND_UV_FLAG, uv_flag, 1 );

        /* Quantization of the subframe gain parameter */
        QuantizeSHBsubgains_fx( st_fx, GainShape, st_fx->extl_fx );
    }

    /* Compute the power of gains away from the peak gain after quantization */
    p2m_out = pow_off_pk_fx( GainShape, NUM_SHB_SUBFR / 2, 2 );

    /* Estimate the gain parameter */
    EstimateSHBFrameGain_fx( SHB_OVERLAP_LEN / 2, hb_frame, Q_ns, shaped_wb_excitation, Q_bwe_exc_ext, GainShape,
                             &GainFrame, window_wb_fx,  subwin_wb_fx );


    /* If there's a big difference in the power of gains away from the peak gain */
    /* due to poor quantization then suppress energy of the high band. */

    IF( L_sub( p2m_out, L_shl( p2m_in, 1 ) ) > 0 )
    {
        L_tmp = root_a_over_b_fx( L_shl( p2m_in, 1 ), 29, p2m_out, 29, &exp_out );
        GainFrame = L_shl( Mult_32_32( GainFrame, L_tmp ), exp_out ); /* Q18 */
    }

    pitBufAvg_fx = 0;
    move16();

    FOR( i = 0; i < NB_SUBFR; i++ )
    {
        pitBufAvg_fx = add( pitBufAvg_fx, mult_r( pitch_buf[i], 82 ) ); /*Q6 */
    }
    voicingBufAvg_fx = 0;
    move16();
    FOR( i = 0; i < 3; i++ )
    {
        voicingBufAvg_fx = add( voicingBufAvg_fx, mult_r( voicing_fx[i], 10912 ) ); /*Q15 */
    }
    /* GainFrame *= max(min((float)(pitBufAvg/voicingBufAvg), 1.0f), 0.7f); */
    IF( voicingBufAvg_fx )
    {
        exp = norm_s( voicingBufAvg_fx );
        tmp = div_s( shl(1,sub( 14,exp )), voicingBufAvg_fx );/* (14-exp) */
        L_tmp = L_mult( pitBufAvg_fx, tmp ); /* (21-exp) */
        L_tmp = L_shl( L_tmp, add(exp, 10) );
        tmp = round_fx( L_tmp ); /* Q15 */
    }
    ELSE
    {
        tmp = 0;
        move16();
    }

    tmp = s_max( s_min( tmp, 32767 ), 22938 ); /* Q15 */
    GainFrame = Mult_32_16( GainFrame, tmp ); /* Q18 */

    IF( sub( lsp_spacing_fx, 328 ) < 0 && lsp_spacing_fx )
    {
        GainFrame = Mult_32_16( GainFrame, 21299 ); /* Q18 */
    }

    IF( sub(st_fx->codec_mode, MODE1) == 0 )
    {
        /*wbbwe_em_factor = add( mult_r( 29491, st_fx->prev_wbbwe_em_factor_fx ), mult_r( 3277, wbbwe_em_factor ) ); */ /* Q15 */
    }


    /*0.25f*sum_f(voice_factors, NB_SUBFR)*/
    L_tmp = L_mult(voice_factors[0], 8192);
    FOR (i=1; i<NB_SUBFR; i++)
    {
        L_tmp = L_mac(L_tmp, voice_factors[i], 8192);
    }
    avg_voice_fac = round_fx(L_tmp);

    test();
    test();
    IF( st_fx->igf != 0 && sub(coder_type, VOICED) == 0 )
    {
        /*GainFrame *= 0.5f;*/

        GainFrame = Mult_32_16( GainFrame, 16384 );
    }
    ELSE IF( st_fx->igf != 0 && sub( avg_voice_fac, 11469 ) > 0 )  /*Q15 -> 0.35f*/
    {
        /*GainFrame *= 0.75f;*/
        GainFrame = Mult_32_16( GainFrame, 24576 );
    }

    /* Quantization of the frame gain parameter */
    QuantizeSHBframegain_fx( st_fx, &GainFrame, st_fx->extl_fx, st_fx->extl_brate_fx, &st_fx->rf_bwe_gainFr_ind );

    /* Adjust the subframe and frame gain of the synthesized SHB signal */
    /* Scale the shaped excitation*/
    ScaleShapedSHB_fx( SHB_OVERLAP_LEN / 2, shaped_wb_excitation, st_fx->syn_overlap_fx, GainShape, GainFrame, window_wb_fx, subwin_wb_fx,
                       &Q_bwe_exc_ext
                       , &dummy
                       , dummy
                       , dummy
                     );

    st_fx->prev_Q_bwe_exc = Q_bwe_exc;
    move16();

    return;
}


void fb_tbe_reset_enc_fx(
    Word32 elliptic_bpf_2_48k_mem_fx[][4],
    Word32 *prev_fb_energy_fx
)
{
    set32_fx( elliptic_bpf_2_48k_mem_fx[0], 0, 4 );
    set32_fx( elliptic_bpf_2_48k_mem_fx[1], 0, 4 );
    set32_fx( elliptic_bpf_2_48k_mem_fx[2], 0, 4 );
    set32_fx( elliptic_bpf_2_48k_mem_fx[3], 0, 4 );
    *prev_fb_energy_fx = 0;
    move16();

    return;
}

/*-------------------------------------------------------------------*
 * fb_bwe_reset_enc()
 *
 * Reset extra parameters needed for FB BWE encoding
 *-------------------------------------------------------------------*/

void fb_bwe_reset_enc_fx(
    Word32 elliptic_bpf_2_48k_mem_fx[][4],
    Word32 *prev_energy_fbe_fb_fx
)
{
    set32_fx( elliptic_bpf_2_48k_mem_fx[0], 0, 4 );
    set32_fx( elliptic_bpf_2_48k_mem_fx[1], 0, 4 );
    set32_fx( elliptic_bpf_2_48k_mem_fx[2], 0, 4 );
    set32_fx( elliptic_bpf_2_48k_mem_fx[3], 0, 4 );
    *prev_energy_fbe_fb_fx = 0;
    move16();

    return;
}


/*======================================================================================*/
/* FUNCTION : void swb_tbe_enc_fx () */
/*--------------------------------------------------------------------------------------*/
/* PURPOSE : SWB TBE encoder, 6 - 14 kHz (or 7.5 - 15.5 kHz) band encoding module */
/*--------------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word16) coder_type_fx : coding type */
/* _(Word16*) shb_speech_fx : SHB target signal (6-14kHz) at 16kHz Q0 */
/* _(Word16*) bwe_exc_extended :bandwidth extended exciatation Q0 */
/* _(Word16[]) voice_factors :voicing factors Q15 */
/* _(Word16*) Q_white_exc :Q Format of White Exc */
/*--------------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16[])White_exc16k_fx : shaped white excitation for the FB TBE Q_white_exc */
/* _(Word16*)fb_slope_fx : slope +ve (high freq > low freq), -ve or neutral Q12 */
/*--------------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ Encoder_State_fx *st_fx: : Encoder state structure */
/*--------------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------------------*/
/* CALLED FROM : TX */
/*======================================================================================*/

void swb_tbe_enc_fx(
    Encoder_State_fx *st_fx,                        /* i/o: encoder state structure */
    const Word16 coder_type_fx,                 /* i : coding type */
    Word16 *shb_speech_fx,                /* i : SHB target signal (6-14kHz) at 16kHz Q_shb*/
    Word32 *bwe_exc_extended,             /* i : bandwidth extended exciatation 2*Q_new*/
    const Word16 voice_factors_fx[],            /* i : voicing factors Q15*/
    Word16 *White_exc16k_fx,              /* o : shaped white excitation for the FB TBE Q_white_exc*/
    Word16 *Q_white_exc,
    Word16 Q_new,
    Word16 Q_shb,
    Word16 *voicing,                      /* i  : OL maximum normalized correlation      */
    const Word16 pitch_buf[]                   /* i : subframe pitch Q6*/
)
{
    Word16 i, j;

    Word16 shb_old_speech_fx[ L_LOOK_16k + L_SUBFR16k + L_FRAME16k ];
    Word16 bwe_exc_extended_16[ L_FRAME32k+NL_BUFF_OFFSET ];

    Word16 shaped_shb_excitation_fx [ L_FRAME16k + L_SHB_LAHEAD ];
    Word32 GainFrame_fx;
    Word16 GainShape_fx[ NUM_SHB_SUBFR ];
    Word16 lpc_shb_fx[ LPC_SHB_ORDER + 1 ], lsp_shb_fx[ LPC_SHB_ORDER ], lsf_shb_fx[ LPC_SHB_ORDER ];
    Word16 weights_lsp[LPC_SHB_ORDER];
    Word16 Q_out;
    Word16* shb_frame_fx, *shb_new_speech_fx;
    Word16 R_h[ LPC_SHB_ORDER+ 2 ];           /* Autocorrelations of windowed speech MSB */
    Word16 R_l[ LPC_SHB_ORDER+ 2 ];           /* Autocorrelations of windowed speech LSB */
    Word16 Q_R;
    Word32 LepsP[LPC_SHB_ORDER+1];

    Word16 ana_align_delay[2] = {-L_SHB_LAHEAD  - (NL_BUFF_OFFSET/2), -L_SHB_LAHEAD  - (NL_BUFF_OFFSET/2)};
    Word32 prev_pow_fx, curr_pow_fx, Lscale;
    Word32 p2m_in_fx, p2m_out_fx;

    Word16 exp_out, exp, exp1, frac;
    Word16 cnt, n_mem, n_mem2;
    Word32 L_tmp, L_tmp1;
    Word16 Q_bwe_exc;

    Word16 frGainAttenuate, frGainSmoothEn;
    Word16 MA_lsp_shb_spacing;
    Word16 temp_swb_fac, feedback;
    Word32 L_feedback;
    Word16 tmp, tmp1, tmp2;
    Word32 Lmax;
    Word16 sc;
    Word16 lsf_shb_orig_fx[LPC_SHB_ORDER];
    Word16 sd_uq_q_fx;
    Word16 vf_modified_fx[NB_SUBFR16k];
    Word16 pitBufAvg_fx;
    Word16 voicingBufAvg_fx;
    Word16 sum1, sum2;
    Word16 recip,Q_recip;
    const Word16 *ptr_lsp_interp_coef_fx;

    Word16 lsp_shb_1_fx[LPC_SHB_ORDER], lsp_shb_2_fx[LPC_SHB_ORDER], lsp_temp_fx[LPC_SHB_ORDER];
    Word16 lpc_shb_sf_fx[4*(LPC_SHB_ORDER+1)];

    /*Word32 shb_ener_sf_fx_32[4];*/
    Word32 shb_ener_sf_Q31;
    Word16 shb_res_fx[L_FRAME16k];
    Word16 shb_res_gshape_fx[NB_SUBFR16k];
    Word32 shb_res_gshape_fx_32[NB_SUBFR16k];
    Word16 vf_ind_fx;

    Word16 formant_fac_fx;
    Word16 shaped_shb_excitationTemp_fx[L_FRAME16k];

    Word16 mean_vf;
    Word16 lsf_diff[LPC_SHB_ORDER], w[LPC_SHB_ORDER];
    Word16 refl[M];
    Word16 tilt_para;
    Word16 Q_bwe_exc_fb;

    /*    init and buffers set up   */
    exp1 = 0;        /* to avoid compilation warnings */
    set16_fx( shaped_shb_excitationTemp_fx, 0, L_FRAME16k );

    /* compensate for the delay in target generation and subframe LA */
    shb_frame_fx = shb_old_speech_fx + L_SUBFR16k + ana_align_delay[0];
    move16();

    /* set up the speech buffers for TBE processing*/
    shb_new_speech_fx = shb_old_speech_fx + (L_LOOK_16k + L_SUBFR16k);
    move16();
    Copy( st_fx->old_speech_shb_fx, shb_old_speech_fx, (L_LOOK_16k + L_SUBFR16k) );
    Copy( shb_speech_fx, shb_new_speech_fx, L_FRAME16k );
    Copy( shb_old_speech_fx + L_FRAME16k, st_fx->old_speech_shb_fx, (L_LOOK_16k + L_SUBFR16k) );

    /* autocorrelation of SHB speech for 10-th order LP analysis */
    autocorr_fx( shb_old_speech_fx,
                 LPC_SHB_ORDER + 1,
                 R_h,    /* autocorr (msb)  Q15 */
                 R_l,    /* autocorr (lsb)      */
                 &Q_R,
                 NS2SA(INT_FS_16k, ACELP_LOOK_NS) + L_SUBFR16k + L_FRAME16k,
                 win_lpc_shb_fx,
                 0,
                 1 );


    /* Spectral smoothing of autocorrelation coefficients */
    test();
    IF( (st_fx->rf_mode != 0) || L_sub( st_fx->total_brate_fx, ACELP_9k60 ) == 0 )
    {
        FOR( i = 1; i <= LPC_SHB_ORDER; i++ )
        {
            L_tmp = Mpy_32( R_h[i], R_l[i], wac_swb_h[i - 1], wac_swb_l[i - 1] );
            L_Extract( L_tmp, &R_h[i], &R_l[i] );
        }
    }

    /* Set the autocorr[0] element to a non-negative value */
    R_l[0] = s_max( R_l[0], 1 );
    move16();

    E_LPC_lev_dur(R_h, R_l, lpc_shb_fx, LepsP, LPC_SHB_ORDER, NULL);   /* LPC in Q14 */
    {
        Word16 enerG, lpc_shb1[M+1];

        /* extend the lpc_shb to a 16th order gain calc */
        set16_fx(lpc_shb1, 0, M+1);
        Copy(lpc_shb_fx, lpc_shb1, LPC_SHB_ORDER + 1);

        /* estimate the LP gain */
        enerG = Enr_1_Az_fx(lpc_shb1, 2*L_SUBFR);   /* Q5 */

        /* if the LP gain is greater than a threshold, avoid saturation */
        IF(sub(enerG, FL2WORD16_SCALE(64, 16 - 5)) > 0)
        {
            set16_fx(lpc_shb_fx, 0, LPC_SHB_ORDER+1);
            E_LPC_lev_dur(R_h, R_l, lpc_shb_fx, LepsP, 2, NULL);   /* LPC in Q14 */
        }
    }

    /* this is needed as the E_LPC_lev_dur function outputs lpc in Q14 */
    Copy_Scale_sig( lpc_shb_fx, lpc_shb_fx, LPC_SHB_ORDER+1, sub(norm_s(lpc_shb_fx[0]),2) );

    /* Expand bandwidth of the LP coeffs */
    test();
    IF( (st_fx->rf_mode != 0) || L_sub( st_fx->total_brate_fx, ACELP_9k60 ) == 0 )
    {
        FOR( i = 1; i <= LPC_SHB_ORDER; i++ )
        {
            lpc_shb_fx[i] = mult_r(lpc_shb_fx[i], lpc_weights_fx[i]);
        }
    }

    /* LPC to LSP conversion */
    /* LPC: Q12, LSP: Q15 */
    E_LPC_a_lsp_conversion(lpc_shb_fx, lsp_shb_fx, st_fx->prev_lsp_shb_fx, LPC_SHB_ORDER );

    /* LSP to LSF conversion */
    /* LSP: Q15, LSF: Q15 */
    E_LPC_lsp_lsf_conversion( lsp_shb_fx, lsf_shb_fx, LPC_SHB_ORDER );

    /* Input signal filtering in case of tonal sounds in the high band
       gain Frame smoothing and attenuation control */
    gainFrSmooth_En_fx(st_fx, shb_frame_fx, lpc_shb_fx, lsf_shb_fx, &MA_lsp_shb_spacing, &frGainAttenuate, &frGainSmoothEn);

    Copy( lsp_shb_fx, st_fx->prev_lsp_shb_fx, LPC_SHB_ORDER );
    Copy( lsf_shb_fx, lsf_shb_orig_fx, LPC_SHB_ORDER );

    test();
    IF( (sub(st_fx->rf_mode,1)==0) || L_sub( st_fx->total_brate_fx, ACELP_9k60 ) == 0 )
    {
        lsp_weights_fx( lsf_shb_fx, weights_lsp, LPC_SHB_ORDER, &Q_out );

        /* to compensate for the 1.1* weighting done inside the function lsp_weights */
        /*weights_lsp[3]*=0.909091f; weights_lsp[4]*=0.909091f; */
        weights_lsp[3] = mult_r( weights_lsp[3], FL2WORD16(0.909091f) );
        weights_lsp[4] = mult_r( weights_lsp[4], FL2WORD16(0.909091f) );

        /* 8-bit VQ, 10 dimension */
        i = closest_centroid_fx( lsf_shb_fx, weights_lsp, swb_tbe_lsfvq_cbook_8b, 256, LPC_SHB_ORDER );
        Copy(swb_tbe_lsfvq_cbook_8b + i*LPC_SHB_ORDER, lsf_shb_fx, LPC_SHB_ORDER);

        set16_fx(st_fx->lsf_idx_fx, 0, NUM_Q_LSF);
        st_fx->lsf_idx_fx[0] = i;
    }
    ELSE
    {
        /* LSF quantization (21 bits) */
        Quant_BWE_LSF_fx( st_fx, lsf_shb_fx, lsf_shb_fx );
    }

    /* space the lsfs to assert a minimum distance */
    space_lsfs_fx( lsf_shb_fx, LPC_SHB_ORDER );

    /* voice factor adjustment and gainframe attenuation factor */
    tmp = sub( lsf_shb_fx[0], lsf_shb_orig_fx[0] );
    L_tmp = L_mult(tmp, tmp);
    FOR( i = 1; i < LPC_SHB_ORDER; i++ )
    {
        /* Estimate the QD in lsfs between UQ and Q */
        tmp = sub( lsf_shb_fx[i], lsf_shb_orig_fx[i] );
        L_tmp = L_mac( L_tmp, tmp, tmp );
    }
    sd_uq_q_fx = round_fx(L_tmp);  /* sd_uq_q_fx in Q15 */
    /* voice factor modification to limit any spurious jumps in the middle of voiced subframes*/
    /* mean(voice_factors_fx[i], 4); */

    L_tmp = L_mult(voice_factors_fx[0], 8192);
    L_tmp = L_mac(L_tmp, voice_factors_fx[1], 8192);
    L_tmp = L_mac(L_tmp, voice_factors_fx[2], 8192);
    mean_vf = mac_r(L_tmp, voice_factors_fx[3], 8192);

    Copy( voice_factors_fx, vf_modified_fx, NB_SUBFR16k );

    test();
    IF( sub(coder_type_fx, VOICED) == 0 || sub(mean_vf, FL2WORD16(0.4f) ) > 0 )
    {
        FOR( i = 1; i < NB_SUBFR; i++ )
        {
            L_tmp = L_mult(voice_factors_fx[i], 26214);
            vf_modified_fx[i] = mac_r(L_tmp, voice_factors_fx[i-1], 6554);
            move16();
        }
        IF( sub(st_fx->L_frame_fx, L_FRAME) != 0 )
        {
            L_tmp = L_mult(voice_factors_fx[4], 26214);
            vf_modified_fx[4] = mac_r(L_tmp, voice_factors_fx[3], 6554);
            move16();
        }
    }

    /* convert quantized LSFs to LSPs for interpolation */
    E_LPC_lsf_lsp_conversion(lsf_shb_fx, lsp_shb_2_fx, LPC_SHB_ORDER);

    test();
    IF( sub(st_fx->last_extl_fx, SWB_TBE) == 0 || sub(st_fx->last_extl_fx, FB_TBE) == 0)
    {
        /* SHB LSP values from prev. frame for interpolation */
        Copy(st_fx->swb_lsp_prev_interp_fx, lsp_shb_1_fx, LPC_SHB_ORDER);
    }
    ELSE
    {
        /* Use current frame's LSPs; in effect no interpolation */
        Copy(lsp_shb_2_fx, lsp_shb_1_fx, LPC_SHB_ORDER);
    }

    lsf_diff[0] = lsf_diff[sub(LPC_SHB_ORDER,1)] = 16384;/*Q15*/
    FOR(i=1; i < LPC_SHB_ORDER-1; i++)
    {
        lsf_diff[i] = sub(lsf_shb_fx[i],lsf_shb_fx[sub(i,1)]);
    }
    a2rc_fx (st_fx->cur_sub_Aq_fx+1, refl,  M);

    /* LSP interpolation for 13.2 kbps and 16.4 kbps */
    /* tilt_para = 6.6956f * (1.0f + refl[0]) * (1.0f + refl[0])
                 - 3.8714f * (1.0f + refl[0])
                 + 1.3041f; */
    tmp = add(16384,shr(refl[0],1));/*Q14*/
    tmp1 = mult(27425 /*Q12*/,tmp);/*Q11*/
    tmp1 = mult(tmp1,tmp);
    tmp2 = shr(mult(31715,tmp),2);  /* Q11 */
    tilt_para = add(sub(tmp1,tmp2),1335);/*Q10*/

    IF(sub(st_fx->last_extl_fx,SWB_TBE) != 0)
    {
        FOR(i=0; i<LPC_SHB_ORDER; i++)
        {
            st_fx->prev_lsf_diff_fx[i] = mult(lsf_diff[i],16384);/*Q15*/
        }
    }

    IF( L_sub(st_fx->total_brate_fx,ACELP_16k40) <= 0 )
    {
        test();
        test();
        test();
        test();
        test();
        IF(!(sub(st_fx->prev_tilt_para_fx,5120) > 0 && (sub(coder_type_fx,TRANSITION) == 0 || sub(tilt_para,1024) < 0)) &&
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
                }
                ELSE
                {
                    tmp = mult(26214,st_fx->prev_lsf_diff_fx[i]);
                    tmp = div_s(tmp,lsf_diff[i]);
                    tmp = s_max(tmp,16384);
                    w[i] = s_min(tmp,32767);
                }
            }
            w[0] = w[1];
            w[sub(LPC_SHB_ORDER,1)] = w[sub(LPC_SHB_ORDER,2)];

            FOR( i = 0; i < LPC_SHB_ORDER; i++ )
            {
                tmp1 = mult(lsp_shb_1_fx[i],sub(32767,w[i]));
                tmp2 = mult(lsp_shb_2_fx[i],w[i]);
                lsp_temp_fx[i] =add(tmp1,tmp2);
            }
        }
        ELSE
        {
            Copy(lsp_shb_2_fx, lsp_temp_fx, LPC_SHB_ORDER);
        }
    }
    Copy(lsf_diff, st_fx->prev_lsf_diff_fx, LPC_SHB_ORDER);
    st_fx->prev_tilt_para_fx = tilt_para;

    shb_ener_sf_Q31 = 0;
    move16();
    test();
    IF ( L_sub(st_fx->total_brate_fx, ACELP_24k40 ) == 0 || L_sub( st_fx->total_brate_fx, ACELP_32k) == 0 )
    {
        /* ---------- SHB LSP interpolation ---------- */
        ptr_lsp_interp_coef_fx = interpol_frac_shb; /* Q15 */
        FOR( j = 0; j < 4; j++ )
        {
            FOR( i = 0; i < LPC_SHB_ORDER; i++ )
            {
                /*lsp_temp_fx[i] =  lsp_shb_1_fx[i]*(*ptr_lsp_interp_coef_fx)       */
                /*                + lsp_shb_2_fx[i]*(*(ptr_lsp_interp_coef_fx+1));  */
                L_tmp = L_mult(lsp_shb_1_fx[i], (*ptr_lsp_interp_coef_fx));
                lsp_temp_fx[i] = mac_r(L_tmp, lsp_shb_2_fx[i], (*(ptr_lsp_interp_coef_fx+1)));
                move16();
            }
            ptr_lsp_interp_coef_fx += 2;

            tmp = i_mult2(j, (LPC_SHB_ORDER+1));
            /* convert LSPs to LPC coefficients for SHB synthesis*/
            E_LPC_f_lsp_a_conversion(lsp_temp_fx, lpc_shb_sf_fx+tmp, LPC_SHB_ORDER);
            /* Bring the LPCs to Q12 */
            Copy_Scale_sig( lpc_shb_sf_fx+tmp, lpc_shb_sf_fx+tmp, LPC_SHB_ORDER+1, sub(norm_s(lpc_shb_sf_fx[tmp]),2) );
        }

        /* -------- Calculate the SHB Energy --------  */
        /*shb_ener_sf[0] = 0.003125f * sum2_f( shb_frame + L_SHB_LAHEAD,     320 );*/
        FOR( i = L_SHB_LAHEAD; i < L_FRAME16k + L_SHB_LAHEAD; i++)
        {
            /* shbEner = shbEner + in[i] * in[i] */
            shb_ener_sf_Q31 = L_mac0(shb_ener_sf_Q31, shb_frame_fx[i], shb_frame_fx[i]);
            /* o: shb_ener_sf_Q31 in (2*Q_shb)      */
        }
        shb_ener_sf_Q31= Mult_32_16(shb_ener_sf_Q31, FL2WORD16(0.003125f));
        shb_ener_sf_Q31= L_add(FL2WORD32_SCALE(1, 31), shb_ener_sf_Q31);
        Quant_shb_ener_sf_fx(st_fx, &shb_ener_sf_Q31, (2*Q_shb));

        /* --------  calculate the residuals using the FOUR subframe LPCs --------  */
        set16_fx(shb_res_fx, 0, L_FRAME16k);
        Residu3_10_fx(lpc_shb_sf_fx,                       shb_frame_fx + L_SHB_LAHEAD,       shb_res_fx,       80, 0);
        Residu3_10_fx(lpc_shb_sf_fx + (LPC_SHB_ORDER+1),   shb_frame_fx + L_SHB_LAHEAD + 80,  shb_res_fx + 80,  80, 0);
        Residu3_10_fx(lpc_shb_sf_fx + 2*(LPC_SHB_ORDER+1), shb_frame_fx + L_SHB_LAHEAD + 160, shb_res_fx + 160, 80, 0);
        Residu3_10_fx(lpc_shb_sf_fx + 3*(LPC_SHB_ORDER+1), shb_frame_fx + L_SHB_LAHEAD + 240, shb_res_fx + 240, 80, 0);
        /* i: shb_frame_fx in Q_shb */
        /* o: shb_res_fx in Q_shb   */

        set32_fx(shb_res_gshape_fx_32, 0, NB_SUBFR16k);
        FOR(i = 0; i < NB_SUBFR16k; i++)
        {
            shb_res_gshape_fx_32[i] = sum2_fx(shb_res_fx+i*64, 64);
        }
        /* o: shb_res_gshape_fx_32  in (2*Q_shb+1) */

        maximum_32_fx(shb_res_gshape_fx_32, NB_SUBFR16k, &L_tmp);

        /* root_a_over_b_fx(shb_res_gshape_fx_32[i], (2*Q_shb+1), L_tmp, (2*Q_shb+1), &exp);*/
        /* First, find 1/L_tmp, L_tmp in QA = (2*Q_shb+1) */

        /* case when den = 0 */
        recip = 0; /*instead of 32767 to be compatible with previous root_a_over_b_fx() output */
        Q_recip = 0;

        IF(L_tmp)
        {
            exp = norm_l(L_tmp);
            tmp = extract_h(L_shl(L_tmp,exp));
            recip = div_s(16384,tmp);
            Q_recip = 31-(exp-14); /* = 31-(exp+2*Q_shb+1-14), but adjusted by (2*Q_shb+1) for use at Mult_32_16 below */
        }

        FOR(i = 0; i < NB_SUBFR16k; i++)
        {
            L_tmp1 = Mult_32_16(shb_res_gshape_fx_32[i], recip); /*Q = Q_recip+1-16*/
            L_tmp = root_a_fx( L_tmp1, Q_recip+1-16, &exp );
            shb_res_gshape_fx[i] = round_fx(L_shl(L_tmp, exp-1)); /* Q14 */
        }

        Quant_shb_res_gshape_fx(st_fx, shb_res_gshape_fx);
    }

    /* for 13.2 and 16.4kbps */
    E_LPC_f_lsp_a_conversion(lsp_temp_fx, lpc_shb_fx, LPC_SHB_ORDER);
    Copy_Scale_sig( lpc_shb_fx, lpc_shb_fx, LPC_SHB_ORDER+1, sub(norm_s(lpc_shb_fx[0]),2) );  /* Q12 */

    /* Save the SWB LSP values from current frame for interpolation */
    Copy(lsp_shb_2_fx, st_fx->swb_lsp_prev_interp_fx, LPC_SHB_ORDER);      /* lsp_shb_2_fx in Q15 */

    /* -------- start of  memory rescaling  -------- */
    /* ----- calculate optimum Q_bwe_exc and rescale memories accordingly ----- */
    Lmax = L_deposit_l(0);
    FOR( cnt = 0; cnt < L_FRAME32k; cnt++ )
    {
        Lmax = L_max( Lmax, L_abs( bwe_exc_extended[cnt] ) );
    }
    Q_bwe_exc = norm_l( Lmax );
    if(Lmax == 0)
    {
        Q_bwe_exc = 31;
    }
    Q_bwe_exc = add( Q_bwe_exc, add( Q_new, Q_new ) );

    /* Account for any outliers in the memories from previous frame for rescaling to avoid saturation */
    find_max_mem_enc( st_fx, &n_mem, &n_mem2);

    tmp2 = add( st_fx->prev_Q_bwe_exc, n_mem );
    if( sub( Q_bwe_exc, tmp2) > 0 )
    {
        Q_bwe_exc = tmp2;
    }

    /* rescale the memories if Q_bwe_exc is different from previous frame */
    sc = sub( Q_bwe_exc, st_fx->prev_Q_bwe_exc );
    IF( sc != 0 )
    {
        rescale_genSHB_mem_enc( st_fx, sc );
    }

    /* rescale the bwe_exc_extended and bring it to 16-bit single precision with dynamic norm  */
    Copy( st_fx->old_bwe_exc_extended_fx, bwe_exc_extended_16, NL_BUFF_OFFSET );
    sc = sub( Q_bwe_exc, add( Q_new, Q_new ) );

    FOR( cnt = 0; cnt < L_FRAME32k; cnt++ )
    {
        bwe_exc_extended_16[cnt +  NL_BUFF_OFFSET] = round_fx( L_shl( bwe_exc_extended[cnt], sc ) );
    }
    Copy( bwe_exc_extended_16 + L_FRAME32k, st_fx->old_bwe_exc_extended_fx, NL_BUFF_OFFSET );

    /* state_syn_shbexc_fx is kept at (st_fx->prev_Q_bwe_syn) for 24.4/32kbps or is kept at Q_bwe_exc for 13.2/16.4kbps */
    Copy( st_fx->state_syn_shbexc_fx, shaped_shb_excitation_fx, L_SHB_LAHEAD );

    /* save the previous Q factor of the buffer */
    st_fx->prev_Q_bwe_exc = Q_bwe_exc;
    move16();

    Q_bwe_exc = sub( Q_bwe_exc, 16 );   /* Q_bwe_exc reflecting the single precision dynamic norm-ed buffers from here */

    /* -------- end of rescaling memories -------- */

    /* Determine formant PF strength */
    formant_fac_fx = swb_formant_fac_fx( lpc_shb_fx[1], &st_fx->tilt_mem_fx );
    /* i:lpc_shb_fx Q12, o:formant_fac_fx Q15 */

    /* Calculate the 6 to 14 kHz (or 7.5 - 15.5 kHz) SHB excitation signal from the ACELP core excitation */
    vf_ind_fx = 20;
    move16();

    GenShapedSHBExcitation_fx( shaped_shb_excitation_fx + L_SHB_LAHEAD, lpc_shb_fx, White_exc16k_fx,
                               st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                               coder_type_fx, bwe_exc_extended_16, st_fx->bwe_seed_fx, vf_modified_fx, st_fx->extl_fx,
                               &( st_fx->tbe_demph_fx ), &( st_fx->tbe_premph_fx ), lpc_shb_sf_fx, shb_ener_sf_Q31,
                               shb_res_gshape_fx, shb_res_fx, &vf_ind_fx, formant_fac_fx, st_fx->fb_state_lpc_syn_fx,
                               &(st_fx->fb_tbe_demph_fx), &Q_bwe_exc,&Q_bwe_exc_fb, Q_shb, n_mem2, st_fx->prev_Q_bwe_syn, st_fx->total_brate_fx);

    *Q_white_exc = Q_bwe_exc_fb;

    test();
    IF( L_sub(st_fx->total_brate_fx, ACELP_24k40) == 0 || L_sub(st_fx->total_brate_fx, ACELP_32k) == 0 )
    {
        IF( sub( st_fx->codec_mode, MODE2 ) == 0 )
        {
            st_fx->idx_mixFac_fx = vf_ind_fx;
            move16();
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_SHB_VF, vf_ind_fx, NUM_BITS_SHB_VF);
        }
    }

    FOR( i = 0; i < LPC_SHB_ORDER; i++ )
    {
        st_fx->mem_stp_swb_fx[i] = shl(st_fx->mem_stp_swb_fx[i], sub(Q_bwe_exc, st_fx->prev_Q_bwe_syn) );
    }

    FOR( i = 0; i < L_FRAME16k; i += L_SUBFR16k )
    {
        PostShortTerm_fx( &shaped_shb_excitation_fx[L_SHB_LAHEAD+i], lpc_shb_fx, &shaped_shb_excitationTemp_fx[i], st_fx->mem_stp_swb_fx,
                          st_fx->ptr_mem_stp_swb_fx, &(st_fx->gain_prec_swb_fx), st_fx->mem_zero_swb_fx, formant_fac_fx );
        /* i: shaped_shb_excitation_fx in Q_bwe_exc */
        /* i: lpc_shb_fx in Q12 */
    }
    Copy( shaped_shb_excitationTemp_fx, &shaped_shb_excitation_fx[L_SHB_LAHEAD], L_FRAME16k );



    tmp = sub(shl(Q_bwe_exc, 1), 31);
    prev_pow_fx = L_shl(FL2WORD32(0.00001f), tmp);   /* 2*(Q_bwe_exc) */
    curr_pow_fx = L_shl(FL2WORD32(0.00001f), tmp);   /* 2*(Q_bwe_exc) */

    FOR( i = 0; i < L_SHB_LAHEAD; i++ )
    {
        prev_pow_fx = L_mac0( prev_pow_fx, shaped_shb_excitation_fx[i], shaped_shb_excitation_fx[i] ); /* 2*Q_bwe_exc */
        curr_pow_fx = L_mac0( curr_pow_fx, shaped_shb_excitation_fx[i + L_SHB_LAHEAD], shaped_shb_excitation_fx[i + L_SHB_LAHEAD] ); /* 2*Q_bwe_exc */
    }

    if( sub( voice_factors_fx[0], FL2WORD16(0.75f) ) > 0 )
    {
        /*curr_pow_fx = Mult_32_16( curr_pow_fx, 8192);*/ /* Q(2*Q_bwe_exc) */
        curr_pow_fx = L_shr(curr_pow_fx, 2); /* scale by 0.25 */
    }

    Lscale = root_a_over_b_fx( curr_pow_fx,
                               shl(Q_bwe_exc, 1),
                               prev_pow_fx,
                               shl(Q_bwe_exc, 1),
                               &exp );

    FOR( i = 0; i < L_SHB_LAHEAD - 1; i++ )
    {
        L_tmp = Mult_32_16( Lscale, shaped_shb_excitation_fx[i] ); /* Q(16-exp+Q_bwe_exc) */
        shaped_shb_excitation_fx[i] = round_fx( L_shl( L_tmp, exp ) ); /* Q_bwe_exc */
    }

    Lscale = root_a_fx( Lscale, 31 - exp, &exp );

    L_tmp = Mult_32_16( Lscale, shaped_shb_excitation_fx[L_SHB_LAHEAD - 1] ); /* Q(16-exp+Q_bwe_exc) */
    shaped_shb_excitation_fx[L_SHB_LAHEAD - 1] = round_fx( L_shl( L_tmp, exp ) ); /* Q_bwe_exc */

    /* Update SHB excitation */
    Copy( shaped_shb_excitation_fx + L_FRAME16k, st_fx->state_syn_shbexc_fx, L_SHB_LAHEAD ); /* Q_bwe_exc */

    /* Estimate the gain-shape parameter */
    EstimateSHBGainShape_fx( SHB_OVERLAP_LEN, shb_frame_fx, Q_shb, shaped_shb_excitation_fx,
                             Q_bwe_exc,  GainShape_fx, subwin_shb_fx );

    /* Gain shape BWS/high band low energy fix */
    IF( sub(st_fx->cldfbHBLT, FL2WORD16_SCALE(1.0f, 15 - 13)) < 0 )   /* cldfbHBLT in Q13 */
    {
        /* There is not much HB past 10kHz; the temporal resolution is quite coarse, so reduce the dynamic range */
        FOR(i = 0; i < NUM_SHB_SUBGAINS; i++)
        {
            /* 0.35f +/- delta variation; delta = 0.1*(GS-0.35)*/
            /* GainShape[i] = 0.315f + 0.1f * GainShape[i]; */
            GainShape_fx[i] = mac_r(FL2WORD32(0.315), FL2WORD16(0.1), GainShape_fx[i]);
        }
    }

    /* Gain frame adjustment factor */
    /* log( (GainShape[0]) / (st->prev_wb_GainShape) )*/
    test();
    IF( GainShape_fx[0] && st_fx->prev_swb_GainShape_fx )
    {
        exp = norm_s( st_fx->prev_swb_GainShape_fx );
        tmp = div_s( shl(1, sub( 14, exp )), st_fx->prev_swb_GainShape_fx );
        L_tmp = L_mult( GainShape_fx[0], tmp );/*Q(30 - exp) */

        exp1 = norm_l( L_tmp );
        frac = Log2_norm_lc( L_shl( L_tmp, exp1 ) );/*move16(); */
        exp1 = sub(exp, exp1 ); /*move16(); */
        L_tmp = Mpy_32_16( exp1, frac, 22713 );
        temp_swb_fac = round_fx( L_shl( L_tmp, 10 ) );
    }
    ELSE
    {
        temp_swb_fac = 0;
        move16();
    }
    L_feedback = L_mult0( temp_swb_fac, temp_swb_fac );


    FOR( i = 1; i < NUM_SHB_SUBGAINS; i++ )
    {
        test();
        IF( GainShape_fx[i] && GainShape_fx[i - 1] )
        {
            exp = norm_s( GainShape_fx[i - 1] );
            tmp = div_s( shl(1, sub( 14, exp )), GainShape_fx[i - 1] );
            L_tmp = L_mult( GainShape_fx[i], tmp );/* Q(30 - exp) */

            exp1 = norm_l( L_tmp );
            frac = Log2_norm_lc( L_shl( L_tmp, exp1 ) );
            exp1 = sub( exp , exp1 );
            L_tmp = Mpy_32_16( exp1, frac, 22713 );
            temp_swb_fac = round_fx( L_shl( L_tmp, 10 ) );
        }
        ELSE
        {
            temp_swb_fac = 0;
            move16();
        }

        L_feedback = L_mac( L_feedback, temp_swb_fac, temp_swb_fac );
    }

    /* feedback = 0.4f / (1 + 0.5f * feedback) */
    L_tmp = L_add( L_shr( L_feedback, 1 ), L_shl(1,21) ); /* Q21 */
    IF( L_tmp != 0 )
    {
        exp = norm_l( L_tmp );
        tmp = extract_h( L_shl( L_tmp, exp ) );
        exp = sub( sub( 30, exp ), 21 );
        tmp = div_s( 16384, tmp ); /*Q(15+exp) */
        L_tmp = L_shr( L_mult( 13107, tmp ), exp ); /*Q31 */
        feedback = round_fx( L_tmp ); /*Q15 */
    }
    ELSE
    {
        feedback = 8738;
        move16(); /*Q15 */
    }

    /* voicing in Q15 */
    L_tmp =       L_mult(voicing[0],8192);
    L_tmp = L_mac(L_tmp, voicing[1],8192);
    L_tmp = L_mac(L_tmp, voicing[2],8192);
    tmp = sum1 = round_fx(L_tmp); /* sum1 used again below - Q13 */
    tmp = add(tmp,1);  /* To ensure change is BE */

    /* voice_factors_fx in Q15 */
    L_tmp =       L_mult(voice_factors_fx[0],8192);
    L_tmp = L_mac(L_tmp, voice_factors_fx[1],8192);
    L_tmp = L_mac(L_tmp, voice_factors_fx[2],8192);
    L_tmp = L_mac(L_tmp, voice_factors_fx[3],8192);
    tmp1 = sum2 = round_fx(L_tmp); /* sum2 used again below - Q13 */


    test();
    test();
    IF( sub(frGainAttenuate,1) == 0 || ( sub( tmp, 19661 ) > 0 &&   sub( tmp1, 6554 ) > 0 ) )

    {
        temp_swb_fac = st_fx->prev_swb_GainShape_fx;
        FOR( i = 0; i < NUM_SHB_SUBGAINS; i++ )
        {
            /*GainShape_fx[i] = (1 - feedback) * GainShape[i] + feedback * temp_swb_fac; */
            GainShape_fx[i] = round_fx(L_mac(L_mult(sub( 32767, feedback ), GainShape_fx[i] ),feedback, temp_swb_fac ) );
            temp_swb_fac = GainShape_fx[i];
            move16();
        }
    }
    st_fx->prev_swb_GainShape_fx = GainShape_fx[3];
    move16();

    /* Compute the power of gains away from the peak gain prior to quantization */
    p2m_in_fx = pow_off_pk_fx( GainShape_fx, NUM_SHB_SUBGAINS, 1 );

    /* Quantization of the gain shape parameter */

    QuantizeSHBsubgains_fx( st_fx, GainShape_fx, st_fx->extl_fx );
    /* o: GainShape_fx in Q15 */
    /* Compute the power of gains away from the peak gain after quantization */
    p2m_out_fx = pow_off_pk_fx( GainShape_fx, NUM_SHB_SUBFR, 4 );

    /* Estimate the gain parameter */
    EstimateSHBFrameGain_fx( SHB_OVERLAP_LEN, shb_frame_fx, Q_shb, shaped_shb_excitation_fx, Q_bwe_exc,
                             GainShape_fx, &GainFrame_fx, window_shb_fx, subwin_shb_fx );

    IF( sub(st_fx->tec_tfa, 1) == 0 )
    {
        tfaCalcEnv_fx( shb_frame_fx, st_fx->tfa_enr );
    }

    /* If there's a big difference in the power of gains away from the peak gain */
    /* due to poor quantization then suppress energy of the high band. */
    IF( L_sub( p2m_out_fx, L_shl( p2m_in_fx, 1 ) ) > 0 )
    {
        L_tmp = root_a_over_b_fx( p2m_in_fx, 28, p2m_out_fx, 29, &exp_out );
        GainFrame_fx = L_shl( Mult_32_32( GainFrame_fx, L_tmp ), exp_out ); /* Q18 */
    }
    test();
    IF( sub(frGainSmoothEn,1) == 0 && L_sub( st_fx->prev_gainFr_SHB_fx, GainFrame_fx ) < 0 )
    {
        GainFrame_fx = L_add( L_shr(st_fx->prev_gainFr_SHB_fx, 1), L_shr(GainFrame_fx, 1) );  /* Q18 */
    }

    test();
    IF( sub(frGainAttenuate, 1) == 0 && sub( MA_lsp_shb_spacing, FL2WORD16(0.0024f) ) <= 0 )
    {
        exp1 = norm_l( GainFrame_fx );
        frac = Log2_norm_lc( L_shl( GainFrame_fx, exp1 ) );
        exp = sub(30, add( exp1, 21 ) );
        L_tmp = Mpy_32_16( exp, frac, 26214 ); /* Q16 */
        frac = L_Extract_lc( L_tmp, &exp );
        GainFrame_fx = Pow2( 30, frac );
        exp = sub( exp, 30);
        GainFrame_fx = L_shl( GainFrame_fx, exp + 18 ); /* Q18 */
    }
    ELSE IF( sub(st_fx->prev_frGainAtten_fx, 1) == 0 &&  L_sub( Mult_32_16( GainFrame_fx, 10923 ), st_fx->prev_gainFr_SHB_fx ) > 0 )
    {
        /*GainFrame *= (0.8f + 0.5f*feedback); */
        tmp = add( 26214, mult_r( feedback, 16384 ) );
        GainFrame_fx = Mult_32_16( GainFrame_fx, tmp ); /* Q18 */
    }
    st_fx->prev_frGainAtten_fx = frGainAttenuate;
    move16();
    st_fx->prev_gainFr_SHB_fx = GainFrame_fx;
    move16(); /* Q18 */


    /* Gain attenuation when the SWB LSF quantization error is larger than a threshold */
    tmp = mult_r(FL2WORD16_SCALE(400, 15-6), sd_uq_q_fx); /* Q6 * Q15 => Q6 */
    IF(sub(st_fx->L_frame_fx, L_FRAME) == 0)
    {
        tmp1 = mult_r(FL2WORD16(0.2f), tmp);  /* Q15, Q6  => Q6 */
        L_tmp = L_msu(FL2WORD32_SCALE(1.0f, 31-13), tmp1, tmp);  /* Q13 */
    }
    ELSE
    {
        tmp1 = mult_r(FL2WORD16(0.1f), tmp);  /* Q15, Q6  => Q6 */
        L_tmp = L_msu(FL2WORD32_SCALE(1.0f, 31-13), tmp1, tmp);  /* Q13 */
    }
    /* sd_uq_q =  max(min(sd_uq_q, 1.0f), 0.5f); */
    L_tmp = L_min(L_tmp, FL2WORD32_SCALE(1.0f, 31-13));
    L_tmp = L_max(L_tmp, FL2WORD32_SCALE(0.5f, 31-13));  /* Q13 */
    /* keep the L_tmp; dont overwrite */

    /* pitBufAvg =  0.0025f * sum_f(pitch_buf, 4); */
    /* pitch_buf: Q6 */
    pitBufAvg_fx =                    mult_r(pitch_buf[0],1311)  ;
    pitBufAvg_fx = add( pitBufAvg_fx, mult_r(pitch_buf[1],1311) );
    pitBufAvg_fx = add( pitBufAvg_fx, mult_r(pitch_buf[2],1311) );
    pitBufAvg_fx = add( pitBufAvg_fx, mult_r(pitch_buf[3],1311) ); /* Q10 */

    /* voicingBufAvg = (sum_f(voice_factors, 4)=sum2 > 0.6f) ? 0.333f : 0.1667f; */
    tmp2 = FL2WORD16(0.1667f);
    if(sub(sum2, FL2WORD16_SCALE(0.6f, 15-13)) > 0)
    {
        tmp2 = FL2WORD16(0.333f);   /* Q15 */
    }
    voicingBufAvg_fx = shl(mult(tmp2, sum1),2); /* Q15 */



    /* max(min((float)(sd_uq_q*pitBufAvg/voicingBufAvg), 1.0f), 0.6f) */
    /* sd_uq_q: Q13, pitBufAvg_fx: Q6, voicingBufAvg_fx: Q15 */

    /* 1/voicingBufAvg_fx */
    exp = norm_s( voicingBufAvg_fx );
    tmp = div_s( shl(1, sub( 14, exp )), voicingBufAvg_fx ); /* (14-exp) */

    /* sd_uq_q*pitBufAvg */
    L_tmp = Mult_32_16(L_tmp, pitBufAvg_fx);   /* Q13 * Q10 + 1 -16 => Q8 */
    L_tmp = Mult_32_16(L_tmp, tmp );   /* Q8 + (14 - exp) - 15 => Q7 - exp */
    tmp = round_fx(L_shl(L_tmp, 31-(7-exp)));   /* Q15 */

    tmp = s_min(tmp, FL2WORD16(1.0f));
    tmp = s_max(tmp, FL2WORD16(0.6f));
    GainFrame_fx = Mult_32_16(GainFrame_fx, tmp);  /* Q18 + Q15 + 1 - 16 : Q18 */

    test();
    IF(sub(st_fx->L_frame_fx, L_FRAME16k) == 0 || sub(st_fx->rf_mode,1) == 0)
    {
        /* Compensate for energy increase mismatch due to memory-less synthesis*/
        GainFrame_fx = Mult_32_16(GainFrame_fx, FL2WORD16(0.85f) );  /* Q18 */
    }

    /* Quantization of the frame gain parameter */
    QuantizeSHBframegain_fx( st_fx, &GainFrame_fx, st_fx->extl_fx, 0, &st_fx->rf_bwe_gainFr_ind );

    /* Adjust the subframe and frame gain of the synthesized SHB signal */
    /* Scale the shaped excitation */
    IF( L_sub(st_fx->extl_fx,FB_TBE) == 0 )
    {
        tmp = norm_l( GainFrame_fx );
        if(GainFrame_fx == 0)
        {
            tmp = 31;
        }
        L_tmp = L_shl(GainFrame_fx,tmp);/* 18 + tmp */

        tmp1 =0;
        FOR( i = 0; i < L_FRAME16k; i++ )
        {
            L_tmp1 = Mult_32_16( L_tmp, GainShape_fx[NUM_SHB_SUBFR * i / L_FRAME16k] ); /* Q : 18 + tmp +15 -15*/
            /*White_exc16k_fx[i] = round_fx( L_shl(Mult_32_16( L_tmp1, White_exc16k_fx[i]),(23 - tmp -*Q_white_exc)) );*/
            /*18 + tmp +*Q_white_exc -15 + 23 - tmp -*Q_white_exc -16  = 10*/
            White_exc16k_fx[i] = round_fx( Mult_32_16( L_tmp1, White_exc16k_fx[i] ) );/* 18 + tmp +*Q_white_exc -15 -16 */
            /*Lmax = L_max(Lmax,White_exc16k_fx[i]);*/
            tmp1 =s_max(tmp1,White_exc16k_fx[i]);
            /*White_exc16k_fx[i] =32767;*/
            /*White_exc16k_fx[i] = shr(White_exc16k_fx[i],5);*/
        }

        *Q_white_exc = *Q_white_exc + 18 + tmp -15 -16;
        tmp = norm_s( tmp1 );
        if(tmp1 == 0)
        {
            tmp = 15;
        }

        FOR(i=0; i<L_FRAME16k; i++)
        {
            White_exc16k_fx[i] = shl(White_exc16k_fx[i],tmp);/* (*Q_white_exc + tmp)*/
            White_exc16k_fx[i] = shr(White_exc16k_fx[i],1);
        }
        *Q_white_exc = *Q_white_exc +tmp -1;
    }

    /* *Q_white_exc = Q_bwe_exc_mod; move16(); output Qwhiteexc_FB from the GenShapedSHB function*/
    st_fx->prev_Q_bwe_syn = Q_bwe_exc;
    move16();

    return;
}


/*==========================================================================*/
/* FUNCTION : static void EstimateSHBFrameGain_fx() */
/*--------------------------------------------------------------------------*/
/* PURPOSE : Estimate the overall gain factor needed to scale */
/* synthesized highband to original highband signal level. */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word16) length : SHB overlap length Q0 */
/* _(Word16*) oriSHB : target original SHB frame Q_oriSHB */
/* _(Word16*) synSHB : shaped SHB excitation Q_synSHB */
/* _(Word16*) win_shb : SHB window Q15 */
/* _(Word16*) subwin_shb : SHB subframe window Q15 */
/* _(Word16) Q_oriSHB : Q format of oriSHB */
/* _(Word16) Q_synSHB : Q format of synSHB */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word32*)GainFrame :estimat of gain frame Q18 */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _None */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------*/

static void EstimateSHBFrameGain_fx(
    const   Word16 length,          /* i : SHB overlap length                   */
    const   Word16* oriSHB,         /* i : target original SHB frame Q_oriSHB   */
    const   Word16 Q_oriSHB,
    const   Word16* synSHB,         /* i : shaped SHB excitation Q_synSHB       */
    const   Word16 Q_synSHB,
    Word16* subgain,        /* i : estimate of gain shape Q15           */
    Word32* GainFrame,      /* o : estimat of gain frame Q18            */
    const   Word16* win_shb,        /* i : SHB window Q15                       */
    const   Word16* subwin_shb      /* i : SHB subframe window Q15              */
)
{
    const Word16* skip;
    Word16 i, j, k, l_shb_lahead, l_frame;
    Word16 join_length, num_join, sig;
    Word32 mod_syn[L_FRAME16k+L_SHB_LAHEAD];
    Word32 oriNrg = 0, synNrg = 0;
    Word16 sum_gain;
    Word32 frame_gain;
    Word32 L_tmp;
    Word16 exp_out;
    Word16 tmp;


    /* initilaization */
    l_frame = L_FRAME16k;
    move16();
    l_shb_lahead = L_SHB_LAHEAD;
    move16();
    skip = skip_bands_SWB_TBE;


    IF( sub(length,SHB_OVERLAP_LEN / 2) == 0 )
    {
        skip = skip_bands_WB_TBE;
        l_frame = L_FRAME16k / 4;
        move16();
        l_shb_lahead = L_SHB_LAHEAD / 4;
        move16();
    }

    /* apply gain for each subframe, and store noise output signal using overlap-add*/
    set32_fx( mod_syn, 0, l_frame + l_shb_lahead );


    IF( sub(length,SHB_OVERLAP_LEN / 2 ) == 0)
    {
        sum_gain = 0;
        move16();
        j = skip[0];
        move16();
        FOR( k = 0; k < length / 2; k++ )
        {
            sum_gain = mult_r( subwin_shb[2 * k + 2], subgain[0] ); /* Q15 */
            mod_syn[j + k] = L_mult( synSHB[j + k], sum_gain );
            move32();/* Q(16+Q_synSHB) */
            mod_syn[j + k + length / 2] = L_mult( synSHB[j + k + length / 2], subgain[0] );
            move32();/* Q(16+Q_synSHB) */
        }

        FOR( i = 1; i < NUM_SHB_SUBFR / 2; i++ )
        {
            j = skip[i];
            move16();
            FOR( k = 0; k < length; k++ )
            {

                L_tmp = L_mult0( subwin_shb[k + 1], subgain[i] ); /* Q30 */
                sum_gain = round_fx( L_mac0( L_tmp, subwin_shb[length - k - 1], subgain[i - 1] ) );/* Q14 */
                mod_syn[j + k] = L_shl( L_mult( sum_gain, synSHB[j + k] ), 1 );
                move32();/* Q(16+Q_synSHB) */
            }
        }
        FOR( k = 0; k < length / 2; k++ )
        {
            j = skip[i];
            move16();
            sum_gain = mult_r( subwin_shb_fx[ length - 2 * k - 2 ], subgain[i - 1] ); /* Q15 */
            mod_syn[ j + k ] = L_mult( synSHB[j + k], sum_gain );
            move32();/* Q(16+Q_synSHB) */
        }
    }
    ELSE
    {
        num_join = NUM_SHB_SUBFR / NUM_SHB_SUBGAINS;
        move16();
        join_length = i_mult2( num_join, length );
        j = 0;
        move16();
        FOR( k = 0;  k < length; k++ )
        {
            sum_gain = mult_r( subwin_shb[k + 1], subgain[0] ); /* Q15 */
            mod_syn[j] = L_mult( synSHB[j], sum_gain );
            move32(); /* Q(16+Q_synSHB) */
            j++;
        }
        FOR( i = 0; i < NUM_SHB_SUBGAINS - 1; i++ )
        {
            FOR( k = 0; k < join_length - length; k++ )
            {
                mod_syn[j] = L_mult( synSHB[j], subgain[i * num_join] );
                move32(); /* Q(16+Q_synSHB) */
                j++;
            }

            FOR( k = 0; k < length; k++ )
            {
                L_tmp = L_mult0( subwin_shb[length - k - 1], subgain[i * num_join] );
                tmp = round_fx( L_mac0( L_tmp, subwin_shb[k + 1], subgain[( i + 1 ) * num_join] ) ); /* Q14 */
                mod_syn[j] = L_shl( L_mult( tmp, synSHB[j] ), 1 );
                move32(); /* Q(16+Q_synSHB) */
                j++;
            }
        }
        FOR( k = 0; k < join_length - length; k++ )
        {
            mod_syn[j] = L_mult( synSHB[j], subgain[( NUM_SHB_SUBGAINS - 1 ) * num_join] );
            move32();/* Q(16+Q_synSHB)*/
            j++;
        }
        FOR( k = 0; k < length; k++ )
        {
            tmp = mult_r( subwin_shb[length - k - 1], subgain[( NUM_SHB_SUBGAINS - 1 ) * num_join] ); /* Q15 */
            mod_syn[j] = L_mult( tmp, synSHB[j] );
            move32();/* Q(16+Q_synSHB ) */
            j++;
        }

    }
    /* adjust frame energy */
    oriNrg = L_deposit_l(0);
    synNrg = L_deposit_l(0);

    FOR( i = 0; i < l_shb_lahead; i++ )
    {
        sig = mult_r( oriSHB[i], win_shb[i] ); /* Q_oriSHB */
        oriNrg = L_mac0( oriNrg, sig, sig ); /* 2*Q_orisHB*/
        sig = round_fx( Mult_32_16( mod_syn[i], win_shb[i] ) ); /*Q_synSHB */
        synNrg = L_mac0( synNrg, sig, sig ); /* 2*Q_synSHB */
    }

    FOR( ; i < l_frame; i++)
    {
        oriNrg = L_mac0( oriNrg, oriSHB[i], oriSHB[i] ); /* 2*Q_oriSHB */
        sig = round_fx( mod_syn[i] ); /* Q_oriSHB */
        synNrg = L_mac0( synNrg, sig, sig ); /* 2*Q_oriSHB */
    }

    tmp = add(l_frame, l_shb_lahead);
    FOR( ; i < tmp; i++)
    {
        sig = mult_r( oriSHB[i], win_shb[l_frame + l_shb_lahead - 1 - i] ); /* Q_oriSHB */
        oriNrg = L_mac0( oriNrg, sig, sig ); /* 2*Q_oriSHB */

        sig = round_fx( Mult_32_16( mod_syn[i], win_shb[l_frame + l_shb_lahead - 1 - i] ) );  /* Q_oriSHB */
        synNrg = L_mac0( synNrg, sig, sig ); /* 2*Q_oriSHB */
    }

    L_tmp = root_a_over_b_fx( oriNrg, 2 * Q_oriSHB, synNrg, 2 * Q_synSHB, &exp_out );
    frame_gain = L_shl( L_tmp, sub( exp_out, 13 ) ); /* Q18 */
    *GainFrame = frame_gain;
    move32();

    return;
}


static Word32 pow_off_pk_fx( Word16 a[], Word16 len, Word16 step )
{
    Word16 i, j = 0;
    Word32 sum, L_tmp;

    sum = L_shr( L_mult0( a[0], a[0] ), 1 ); /* Q29 */

    j = 0;
    move16();
    FOR( i=1; i < len; i += step)
    {
        L_tmp = L_shr( L_mult0( a[i], a[i] ), 1 ); /* Q29 */
        sum = L_add( sum, L_tmp ); /* Q29 */
        if( sub( a[i], a[j] ) > 0 )
        {
            j = i;
            move16();
        }
    }
    L_tmp = L_shr( L_mult0( a[j], a[j] ), 1 ); /* Q29 */
    sum = L_sub( sum, L_tmp ); /* Q29 */

    return ( sum );
}


/*==========================================================================*/
/* FUNCTION : static void EstimateSHBGainShape_fx() */
/*--------------------------------------------------------------------------*/
/* PURPOSE : Estimate temporal gain parameters */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word16) length : SHB overlap length Q0 */
/* _(Word16*) oriSHB : target original SHB frame Q_oriSHB */
/* _(Word16*) synSHB : shaped SHB excitation Q_synSHB */
/* _(Word16*) subwin : SHB subframe window Q15 */
/* _(Word16) Q_oriSHB : Q format of oriSHB */
/* _(Word16) Q_synSHB : Q format of synSHB */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16*)subgain :estimate of gain shape Q15 */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _None */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------*/
static void EstimateSHBGainShape_fx(
    const  Word16 length,              /* i : SHB overlap length */
    const  Word16* oriSHB,             /* i : target original SHB frame Q_oriSHB*/
    const  Word16 Q_oriSHB,
    const  Word16* synSHB,             /* i : shaped SHB excitation Q_synSHB*/
    const  Word16 Q_synSHB,
    Word16* subgain,            /* o : estimate of gain shape Q15*/
    const  Word16* subwin              /* i : SHB subframe window Q15*/
)
{
    const Word16* skip;
    Word16 i, j, k;
    Word16 sig;
    Word32 L_subgain[NUM_SHB_SUBFR];
    Word32 L_sum_gain   = 0;
    Word32 oriNrg, synNrg;
    Word16 num_join, num_gains, join_length;
    Word16 norm[NUM_SHB_SUBFR];
    Word16 n_max = -32768;
    Word16 n;
    Word16 length2 = shl(length, 1);
    Word16 length_tmp;
    Word32 L_tmp, normFact;

    num_join = NUM_SHB_SUBFR / NUM_SHB_SUBGAINS;
    move16();
    num_gains = NUM_SHB_SUBGAINS;
    move16();
    skip = skip_bands_SWB_TBE;

    IF( sub(length,SHB_OVERLAP_LEN / 2) == 0 )
    {
        num_gains = NUM_SHB_SUBFR / 4;
        move16();
        skip = skip_bands_WB_TBE;
        move16();
    }
    /* calculate and normalize the subgain */
    oriNrg = 0;
    move16();
    synNrg = 0;
    move16();

    IF( sub(length,SHB_OVERLAP_LEN / 2) == 0 )
    {
        FOR( i = 0; i < NUM_SHB_SUBFR / 2; i++ )
        {
            logic16();
            IF( ( i & 0x1 ) == 0 )
            {
                oriNrg = 0;
                move16();
                synNrg = 0;
                move16();
            }
            j = skip[i];
            IF( i == 0 )
            {
                FOR( k = 0; k < length / 2; k++ )
                {
                    sig = mult_r( oriSHB[j + k], subwin[2 * k + 2] ); /* Q_oriSHB */
                    oriNrg = L_mac0( oriNrg, sig, sig ); /* 2* Q_oriSHB */
                    sig = mult_r( synSHB[j + k], subwin[2 * k + 2] ); /* Q_synSHB */
                    synNrg = L_mac0( synNrg, sig, sig ); /* 2* Q_synSHB */
                }
                FOR( k = length / 2; k < length; k++ )
                {
                    sig = oriSHB[j + k];
                    move16();
                    oriNrg = L_mac0( oriNrg, sig, sig ); /* 2* Q_oriSHB */
                    sig = synSHB[j + k];
                    move16();
                    synNrg = L_mac0( synNrg, sig, sig ); /* 2* Q_synSHB */
                }
            }
            ELSE
            {
                FOR( k = 0; k < length; k++ )
                {
                    sig = mult_r( oriSHB[j + k], subwin[k + 1] );   /* Q_oriSHB */
                    oriNrg = L_mac0( oriNrg, sig, sig );            /* 2* Q_oriSHB */

                    sig = mult_r( synSHB[j + k], subwin[k + 1] );   /* Q_synSHB */
                    synNrg = L_mac0( synNrg, sig, sig );            /* 2* Q_synSHB */

                }
            }
            IF( i == NUM_SHB_SUBFR / 2 - 1 )
            {
                length_tmp = sub(length2, shr(length, 1));
                FOR( ; k < length_tmp; k++ )
                {
                    sig = mult_r( oriSHB[j + k], subwin[3 * length - 2 * k - 2] );  /* Q_oriSHB */
                    oriNrg = L_mac0( oriNrg, sig, sig );                            /* 2* Q_oriSHB */

                    sig = mult_r( synSHB[j + k], subwin[3 * length - 2 * k - 2] );  /* Q_synSHB */
                    synNrg = L_mac0( synNrg, sig, sig );                            /* 2* Q_synSHB */
                }
            }
            ELSE
            {
                FOR(; k < length2; k++ )
                {
                    sig = mult_r( oriSHB[j + k], subwin[2 * length - k - 1] ); /* Q_oriSHB */
                    oriNrg = L_mac0( oriNrg, sig, sig ); /* 2* Q_oriSHB */

                    sig = mult_r( synSHB[j + k], subwin[2 * length - k - 1] ); /* Q_synSHB */
                    synNrg = L_mac0( synNrg, sig, sig ); /* 2* Q_synSHB */
                }
            }

            logic16();
            IF( sub(( i & 0x1 ),1) == 0 )
            {
                L_subgain[i / 2] = root_a_over_b_fx( oriNrg, 2 * Q_oriSHB, synNrg, 2 * Q_synSHB, &n );
                move16(); /* Q(31-n) */
                norm[i / 2] = n;
                move16();
                IF( sub(norm[i / 2],n_max) > 0 )
                {
                    n_max = norm[i / 2];
                    move16();
                }
            }
        }
    }
    ELSE
    {
        join_length = i_mult2( num_join, length );
        FOR( i = 0; i < num_gains; i++ )
        {
            oriNrg = 0;
            move16();
            synNrg = 0;
            move16();

            j = join_length * i;
            move16();
            FOR( k = 0; k < length; k++ )
            {
                sig = mult_r( oriSHB[j + k], subwin[k + 1] ); /* Q_oriSHB */
                oriNrg = L_mac0( oriNrg, sig, sig ); /* 2* Q_oriSHB */

                sig = mult_r( synSHB[j + k], subwin[k + 1] ); /* Q_oriSHB */
                synNrg = L_mac0( synNrg, sig, sig ); /* 2* Q_oriSHB */
            }

            FOR( k = 0; k < ( join_length - length ); k++ )
            {
                sig = mult_r( oriSHB[length + j + k], 32767 ); /* Q_oriSHB */
                oriNrg = L_mac0( oriNrg, sig, sig ); /* 2* Q_oriSHB */

                sig = mult_r( synSHB[length + j + k], 32767 ); /* Q_oriSHB */
                synNrg = L_mac0( synNrg, sig, sig ); /* 2* Q_oriSHB */
            }

            FOR( k = 0; k < length; k++ )
            {
                sig = mult_r( oriSHB[j + join_length + k], subwin[length -  k - 1] ); /* Q_oriSHB */
                oriNrg = L_mac0( oriNrg, sig, sig ); /* 2* Q_oriSHB */

                sig = mult_r( synSHB[j + join_length + k], subwin[length -  k - 1] ); /* Q_oriSHB */
                synNrg = L_mac0( synNrg, sig, sig ); /* 2* Q_oriSHB */
            }

            L_subgain[i] = root_a_over_b_fx( oriNrg, 2 * Q_oriSHB, synNrg, 2 * Q_synSHB, &n );
            move32(); /* Q(31-n) */
            norm[i] = n;
            move16();
            IF( sub(norm[i],n_max) > 0 )
            {
                n_max = norm[i];
                move16();
            }
        }
    }

    FOR( i = 0; i < num_gains; i++ )
    {
        subgain[i] = round_fx( L_shl( L_subgain[i], sub( norm[i], n_max + 1 ) ) ); /* Q(14-n_max) */
        L_sum_gain = L_mac0( L_sum_gain, subgain[i], subgain[i] ); /* Q(28-2*n_max) */
    }

    /* normalize the subgain */
    n = norm_l( L_sum_gain );
    L_sum_gain = L_shl( L_sum_gain, n );
    n = sub(31, add (n, (sub( 28 ,  n_max <<1 ) )));
    normFact = Isqrt_lc( L_sum_gain, &n );

    FOR( i = 0; i < num_gains; i++ )
    {
        L_tmp = Mult_32_16( normFact, subgain[i] );  /*Q(31-n) * Q(31-norm[i])      */ /* Q(30-n-n_max) */
        subgain[i] = s_max( round_fx( L_shl( L_tmp, add( n, n_max + 1 ) ) ), FL2WORD16(0.1f)); /* Q15 */
    }

    return;
}



/*==========================================================================*/
/* FUNCTION :static short closest_centroid_fx () */
/*--------------------------------------------------------------------------*/
/* PURPOSE : Determine a set of closest VQ centroids for a given input*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word16*) data : input data Q15 */
/* _(Word16*) weights : weights Q0 */
/* _(Word16*) quantizer : quantizer table Q15 */
/* _(Word16) centroids : number of centroids Q0 */
/* _(Word16) length : dimension of quantiser Q0 */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _None */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _None */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ Word16 : index Q0 */
/*--------------------------------------------------------------------------*/
static Word16 closest_centroid_fx(
    const Word16*   data,           /* i : input data Qx*/
    const Word16*   weights,        /* i : weights */
    const Word16*   quantizer,      /* i : quantizer table Qx*/
    const Word16    centroids,      /* i : number of centroids */
    const Word16    length )        /* i : dimension of quantiser */
{
    Word16 i, j, index;
    Word16 tmp, tmpL;
    Word32 werr, best_werr;
    Word32 L_tmp;

    index = 0;
    move16();
    best_werr = L_add(MAX_32, 0);

    FOR( i = 0; i < centroids; i++ )
    {
        werr = L_deposit_l(0);
        tmpL = i_mult2(i, length);
        FOR( j = 0; j < length; j++ )
        {
            tmp = sub( data[j], quantizer[tmpL + j] );
            L_tmp = L_mult( tmp, tmp );
            werr = Madd_32_16( werr, L_tmp, weights[j] );
        }
        if( L_sub( werr, best_werr ) < 0 )
        {
            index = i;
            move16();
        }
        best_werr = L_min(best_werr, werr);

    }

    return index;
}

/*==========================================================================*/
/* FUNCTION :static short closest_centroid_lc_fx () */
/*--------------------------------------------------------------------------*/
/* PURPOSE : Determine a set of closest VQ centroids for a given input      */
/*           Gain shape is 4 dimensional                                    */
/*--------------------------------------------------------------------------*/
static Word16 closest_centroid_lc_fx(
    const Word16 *data,          /* i : input data Qx*/
    const Word16 *quantizer,     /* i : quantizer table Qx*/
    const Word16 centroids)      /* i : number of centroids */
{
    Word16 i, index, tmp, tmpL;
    Word32 werr, best_werr;

    index = 0;
    move16();
    best_werr = MAX_32;
    move32();

    FOR( i = 0; i < centroids; i++ )
    {
        /* Gain shape dimension 4 */
        tmpL = shl(i, 2);

        /* index 0 */
        tmp = sub( data[0], quantizer[tmpL] );
        werr = L_mult(tmp, tmp);

        /* index 1 */
        tmp = sub( data[1], quantizer[tmpL + 1] );
        werr = L_mac(werr, tmp, tmp);

        /* index 2 */
        tmp = sub( data[2], quantizer[tmpL + 2] );
        werr = L_mac(werr, tmp, tmp);

        /* index 3 */
        tmp = sub( data[3], quantizer[tmpL + 3] );
        werr = L_mac(werr, tmp, tmp);

        if( L_sub( werr, best_werr ) < 0 )
        {
            index = i;
            move16();
        }
        best_werr = L_min(best_werr, werr);
    }

    return index;
}

/*============================================================*/
/* FUNCTION : static void QuantizeSHBsubgains_fx() */
/*------------------------------------------------------------*/
/* PURPOSE : Quantize super highband temporal gains */
/*------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word16) extl : extension layer Q0 */
/*------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _ None */
/*------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _(Word16[])subgains :super highband temporal gains Q15 */
/*------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*------------------------------------------------------------*/

static void QuantizeSHBsubgains_fx(
    Encoder_State_fx *st_fx,  /* i/o: encoder state structure */
    Word16 subgains[],        /* i/o: super highband temporal gains Q15*/
    const Word16 extl               /* i : extension layer */
)
{
    Word16 i, idxSubGain;
    Word16 Unit_weights10[NUM_SHB_SUBFR];
    Word16 exp, frac;
    Word32 L_tmp;

    IF( sub(extl, WB_TBE) == 0 )
    {
        set16_fx( Unit_weights10, 32767, ( Word16 )NUM_SHB_SUBFR / 4 );
        FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
        {
            IF( subgains[i] == 0 )
            {
                subgains[i + NUM_SHB_SUBFR / 4] = -18432;
                move16(); /* (-72) in Q8 */
            }
            ELSE
            {
                L_tmp = L_deposit_h( subgains[i] ); /* Q31 */
                exp = norm_l( L_tmp );
                frac = Log2_norm_lc( L_shl( L_tmp, exp ) ); /* move16(); */
                /*exp = -1 - exp;  */
                exp = sub(-1, exp);
                L_tmp = Mpy_32_16( exp, frac, 24660 );/* Q13 ; 20.0 * log10(2) in Q12*/
                subgains[i + NUM_SHB_SUBFR / 4] = round_fx( L_shl( L_tmp, 11 ) ); /* Q8 */
            }
        }
        idxSubGain = closest_centroid_lc_fx( subgains + NUM_SHB_SUBFR / 4, HBCB_SubGain5bit_fx, 1 << NUM_BITS_SHB_SUBGAINS );
        Copy( HBCB_SubGain5bit_fx + idxSubGain * NUM_SHB_SUBFR / 4, subgains, NUM_SHB_SUBFR / 4 );

        push_indice_fx(  st_fx, IND_SHB_SUBGAIN, idxSubGain, NUM_BITS_SHB_SUBGAINS );

        FOR( i = 0; i < NUM_SHB_SUBFR / 4; i++ )
        {
            L_tmp = L_mult( subgains[i], 21771 ); /* *0.166096 in Q17 -> Q26 */
            L_tmp = L_shr( L_tmp, 10 ); /* From Q26 to Q16 */
            frac = L_Extract_lc( L_tmp, &exp );
            subgains[i] = extract_l( Pow2( 14, frac ) );
            /* Put 14 as exponent so that */
            /* output of Pow2() will be: */
            /* 16384 < Pow2() <= 32767 */
            subgains[i] = shl( subgains[i], exp + 1 );
            move16();/*Q15 */
        }

        FOR( i = NUM_SHB_SUBFR / 2 - 1; i >= 0; i-- )
        {
            subgains[i] = subgains[i / 2];
            move16();
        }
    }
    ELSE
    {
        FOR( i = 0; i < NUM_SHB_SUBGAINS; i++ )
        {
            IF( subgains[i] == 0 )
            {
                subgains[i] = -12288;
                move16(); /* (-3) in Q12 */
            }
            ELSE
            {
                L_tmp = L_deposit_h( subgains[i] ); /* Q31 */
                exp = norm_l( L_tmp );
                frac = Log2_norm_lc( L_shl( L_tmp, exp ) );
                /*exp = -1 - exp;  */
                exp = sub(-1, exp);

                L_tmp = Mpy_32_16( exp, frac, 9864 ); /*move32(); // Q16 ; log10(2) in Q15 */
                subgains[i] = round_fx( L_shl( L_tmp, 12 ) ); /*Q12 */
            }
        }

        idxSubGain = ( Word16 )vquant_fx( subgains, 0, subgains, SHBCB_SubGain5bit_12_fx,  NUM_SHB_SUBGAINS, 1 << NUM_BITS_SHB_SUBGAINS );

        FOR( i = 0; i < NUM_SHB_SUBGAINS; i++ )
        {
            L_tmp = L_mult( subgains[i], 27213 ); /* *3.321928 in Q13 -> Q26 */
            L_tmp = L_shr( L_tmp, 10 ); /* From Q26 to Q16 */
            frac = L_Extract_lc( L_tmp, &exp );
            subgains[i] = extract_l( Pow2( 14, frac ) );
            /* Put 14 as exponent so that */
            /* output of Pow2() will be: */
            /* 16384 < Pow2() <= 32767 */
            subgains[i] = shl( subgains[i], exp + 1 );
            move16();/*Q15 */
        }

        FOR( i = NUM_SHB_SUBFR - 1; i >= 0; i-- )
        {
            subgains[i] = subgains[i * NUM_SHB_SUBGAINS / NUM_SHB_SUBFR];
            move16();
        }

        st_fx->idxSubGains_fx = idxSubGain;
        move16();
        IF( sub( st_fx->codec_mode, MODE2 ) != 0 )
        {
            push_indice_fx( st_fx, IND_SHB_SUBGAIN, idxSubGain, NUM_BITS_SHB_SUBGAINS );
        }
    }

    return;
}



/*-------------------------------------------------------------------*
 * Quant_shb_ener_sf_fx_fx()
 *
 * Quantize SHB subframe energies
 *-------------------------------------------------------------------*/

static void Quant_shb_ener_sf_fx(
    Encoder_State_fx *st_fx,                /* i/o: encoder state structure      */
    Word32 *shb_ener_sf_Q31,      /* i/o: super highband subframe energies  */
    Word16 Q_ener
)
{
    Word16 idxSubEner_fx;
    Word16 temp_shb_ener_sf_fx;
    Word16 exp/*, exp2*/,frac;
    Word32 L_tmp1, L_tmp;
    Word32 sum;
    Word16 tmp;

    /* shb_ener_sf_fx[0] = log10(0.003125*shb_ener_sf[0:319]); */
    sum = *shb_ener_sf_Q31; /* L_tmp in Q_ener = (2*Q_shb+1) */

    exp = norm_l(sum);
    frac = Log2_norm_lc(L_shl(sum, exp));
    exp = sub(30, add(exp, Q_ener));      /* 30-(exp+Q_ener ) */
    L_tmp1 = Mpy_32_16(exp, frac, 617);   /* 2466=LOG10(2) in Q11, so answer Ltmp in Q12 */

    tmp = round_fx(L_shl(L_tmp1, 30-14)); /* tmp in Q12 */

    temp_shb_ener_sf_fx = 0;
    move16();
    idxSubEner_fx  = usquant_fx(tmp, &temp_shb_ener_sf_fx, 0, 86, shl(1,NUM_BITS_SHB_ENER_SF)); /* 86 = 0.042f in Q11 = Qin-1 */
    /* o: temp_shb_ener_sf_fx in Q12 */

    /* shb_ener_sf_fx[0] = pow(10.0, temp_shb_ener_sf_fx );     */
    /*                   = pow(2, 3.321928*temp_shb_ener_sf_fx) */
    L_tmp = L_mult(temp_shb_ener_sf_fx, 27213 );    /* 3.321928 in Q13 -> L_tmp in Q12+Q13+1 = Q26 */
    L_tmp = L_shl( L_tmp, -10 );                    /* bring L_tmp from Q26 to Q16 */
    frac = L_Extract_lc( L_tmp, &exp );             /* Extract exponent */
    L_tmp = Pow2(14, frac );
    *shb_ener_sf_Q31 = L_shl(L_tmp, exp-14+Q_ener ); /* In Q_ener */

    st_fx->idx_shb_fr_gain_fx = idxSubEner_fx;
    move16();
    IF( sub( st_fx->codec_mode, MODE2 ) != 0 )
    {
        push_indice_fx( st_fx, IND_SHB_ENER_SF, idxSubEner_fx, NUM_BITS_SHB_ENER_SF);
    }
    return;
}


/*-------------------------------------------------------------------*
* Quant_shb_res_gshape_fx()
*
* Quantize SHB gain shapes in residual domain
*-------------------------------------------------------------------*/

static void Quant_shb_res_gshape_fx(
    Encoder_State_fx *st_fx,                   /* i/o: encoder state structure      */
    Word16 shb_res_gshape_fx[]         /* i/o: super highband gain shapes  Q14 */
)
{
    Word16 i, idxSubGain_fx[NB_SUBFR16k];

    FOR(i = 0; i < NB_SUBFR16k; i++)
    {
        idxSubGain_fx[i] = usquant_fx(shb_res_gshape_fx[i],
                                      &shb_res_gshape_fx[i],
                                      FL2WORD16_SCALE(0.125f, 15-14), /*2048 = 0.125 in Q14 */
                                      FL2WORD16_SCALE(0.125f, 15-13), /*1024 = 0.125 in Q13 */
                                      shl(1,NUM_BITS_SHB_RES_GS));

        st_fx->idx_res_gs_fx[i] = idxSubGain_fx[i];
        IF ( sub( st_fx->codec_mode, MODE2 ) != 0 )
        {
            push_indice_fx( st_fx, IND_SHB_RES_GS1+i, idxSubGain_fx[i], NUM_BITS_SHB_RES_GS);
        }
    }
}

/*==========================================================================*/
/* FUNCTION : static void QuantizeSHBframegain_fx() */
/*--------------------------------------------------------------------------*/
/* PURPOSE : QQuantize super highband frame gain */
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* Word16 extl i : extension layer */
/* Word32 extl_brate i : extension layer bitrate */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* Word32 *GainFrame i/o: Gain Q18 */
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*--------------------------------------------------------------------------*/
/* */
/*==========================================================================*/
static void QuantizeSHBframegain_fx(
    Encoder_State_fx* st_fx,        /* i/o: encoder state structure */
    Word32* GainFrame,              /* i/o: Gain Q18 */
    const Word16 extl,              /* i : extension layer */
    Word32 extl_brate               /* i : extension layer bitrate */
    ,Word16 *rf_gainFrame_ind
)
{
    Word16 idxFrameGain;
    Word32 Q_GainFrame;
    Word16 Unit_weights1 = 1;
    Word16 exp, frac, tmp;
    Word32 L_tmp;
    Word32 GainFrameLog;

    test();
    IF( sub(extl, WB_TBE) == 0  ||  sub(extl, WB_TBE_0k65) == 0 )
    {
        determine_gain_weights_fx( GainFrame, &( Unit_weights1 ), 1 );
        IF( L_sub(extl_brate,WB_TBE_0k35) == 0 )
        {
            singlevectortest_gain_fx( GainFrame, 1,
                                      1 << NUM_BITS_SHB_FrameGain_LBR_WB, &idxFrameGain,
                                      &Q_GainFrame, SHBCB_FrameGain16_fx );
            test();
            IF( L_sub( Q_GainFrame, L_shl( Mult_32_16( *GainFrame, 17367 ), 1 ) ) > 0 && idxFrameGain > 0 ) /* 1.06 = +0.5 dB */
            {
                idxFrameGain--;
                Q_GainFrame = L_add(SHBCB_FrameGain16_fx[idxFrameGain], 0); /* Q18 */
            }
            st_fx->gFrame_WB_fx = idxFrameGain;
            move16();
            IF( sub( st_fx->codec_mode, MODE2) != 0 )
            {
                push_indice_fx( st_fx, IND_SHB_FRAMEGAIN, idxFrameGain, NUM_BITS_SHB_FrameGain_LBR_WB );
            }
            *rf_gainFrame_ind = idxFrameGain;
        }
        ELSE
        {
            singlevectortest_gain_fx( GainFrame, 1,
            1 << NUM_BITS_SHB_FrameGain, &idxFrameGain, &Q_GainFrame,
            SHBCB_FrameGain64_fx );

            push_indice_fx( st_fx, IND_SHB_FRAMEGAIN, idxFrameGain, NUM_BITS_SHB_FrameGain );
            *rf_gainFrame_ind = idxFrameGain;
            move16();  /* Q18 */
        }
    }
    ELSE
    {
        IF( *GainFrame == 0 )
        {
            GainFrameLog = -196608;
            move32();
        }
        ELSE
        {
            exp = norm_l( *GainFrame );
            frac = Log2_norm_lc( L_shl( *GainFrame, exp ) );
            exp = ( 30 - exp - 18 );
            GainFrameLog = Mpy_32_16( exp, frac, 9864 );
            /*GainFrameLog= round_fx(L_shl(L_tmp,12)); //Q12 */
        }

        exp = norm_s( SHB_GAIN_QDELTA_FX_15 );
        tmp = div_s( shl(1, sub(14, exp)), SHB_GAIN_QDELTA_FX_15 );
        L_tmp = Mult_32_16( L_sub( GainFrameLog, SHB_GAIN_QLOW_FX_16 ), tmp );
        idxFrameGain = extract_l( L_shr( L_add( L_tmp, shl(1, sub(14, exp)) ), sub(15, exp) ) ); /*Q0*/
        IF( sub( idxFrameGain, ( 1 << NUM_BITS_SHB_FRAMEGAIN ) - 1 ) > 0 )
        {
            idxFrameGain = sub( ( 1 << NUM_BITS_SHB_FRAMEGAIN ), 1 );
        }
        ELSE
        {
            if( idxFrameGain < 0 )
            {
                idxFrameGain = 0;
                move16();
            }
        }

        L_tmp = SHB_GAIN_QLOW_FX_16;
        Q_GainFrame = L_mac( L_tmp, idxFrameGain, SHB_GAIN_QDELTA_FX_15 );

        WHILE( L_sub( Q_GainFrame, L_add( GainFrameLog, 4866 ) ) > 0 &&
        idxFrameGain != 0 )
        {
            idxFrameGain = sub(idxFrameGain, 1);
            Q_GainFrame = L_mac0( SHB_GAIN_QLOW_FX_16, idxFrameGain, SHB_GAIN_QDELTA_FX_16 );
        }

        /* Q_GainFrame = (float) pow(10.0, Q_GainFrame ); */
        /* i: Q_GainFrame in Q16 */
        L_tmp = Mult_32_16( Q_GainFrame, 27213 ); /* *3.321928 in Q13 -> Q25 */
        L_tmp = L_shr( L_tmp, -2 ); /* From Q26 to Q16 */
        frac = L_Extract_lc( L_tmp, &exp );
        Q_GainFrame = Pow2( 30, frac );
        exp = sub ( exp, 30) ;
        Q_GainFrame = L_shl( Q_GainFrame, exp + 18 ); /* Q18 */

        st_fx->idxFrameGain_fx = idxFrameGain;
        move16();
        IF ( sub( st_fx->codec_mode, MODE2 ) != 0 )
        {
            push_indice_fx( st_fx, IND_SHB_FRAMEGAIN, idxFrameGain, NUM_BITS_SHB_FRAMEGAIN );
        }
        *rf_gainFrame_ind = idxFrameGain;
    }

    IF( sub(st_fx->rf_mode,1)==0)
    {
        /*Currently intended for SWB only. Modify for WB is needed later!*/
        IF( sub(st_fx->rf_frame_type,RF_NELP) == 0)
        {
            *rf_gainFrame_ind = idxFrameGain; /* NELP Frame uses full 5 bits */
        }
        ELSE /*RF_ALLPRED, RF_GENPRED, RF_NOPRED modes*/
        {
            IF( *GainFrame <= 327680     /*1.25 in Q18*/ )  /* [0 to 1.25] range --> 0.5*/
            *rf_gainFrame_ind = 0;
            ELSE IF ( *GainFrame <= 786432  /*3 in Q18*/ )  /* (1.25 to 3] --> 2 */
            *rf_gainFrame_ind = 1;
            ELSE IF ( *GainFrame <= 1572864 /*6 in Q18*/ )  /* (3 to 6] --> 4 */
            *rf_gainFrame_ind = 2;
            ELSE                        /* (6 to Inf) --> 8 */
            *rf_gainFrame_ind = 3;
        }
    }

    *GainFrame = Q_GainFrame;
    move32();  /* Q18 */

    return;
}


/*============================================================*/
/* FUNCTION : static void determine_gain_weights_fx() */
/*------------------------------------------------------------*/
/* PURPOSE : Determine weights for gain quantization */
/*------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word32*)gain :Gain parameter Q18 */
/* _(Word16) dims : number of gains */
/*------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16*)weights : gain weights Q12/Q6 */
/*------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _ None */
/*------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*------------------------------------------------------------*/
static void determine_gain_weights_fx (
    const   Word32* gain,     /* i : Gain parameter Q18 */
    Word16* weights, /* o : gain weights Q12/Q6*/
    const   Word16 dims /* i : number of gains */ )
{
    Word16 j;
    Word16 exp, exp1, frac, tmp, exp2;
    Word32 L_tmp;

    FOR( j = 0; j < dims; j++ )
    {
        IF( L_sub(gain[j],8) > 0 )/* 8 = 0.001 in Q13 */
        {
            /*weights[j] = (float)(pow (fabs (gain[j]), -0.9f));  = pow(2,(-0.9)*log2(gain[j]))           */
            exp1 = norm_l( gain[j] );
            frac = Log2_norm_lc( L_shl( gain[j], exp1 ) );
            move16();
            exp = 30 - exp1 - 18;
            move16();
            L_tmp = Mpy_32_16( exp, frac, -29491 ); /* Q16 */
            frac = L_Extract_lc( L_tmp, &exp );
            tmp = extract_l( Pow2( 14, frac ) ); /* Q14 */

            exp2 = sub(exp,8);
            if( sub(exp1,21) <= 0 )
            {
                exp2 = sub(exp,2);
            }
            weights[j] = shl( tmp, exp2);
            move16(); /* Q12 */
        }
        ELSE
        {
            weights[j] = 32076;
            move16();/* (501.187233628f) in Q6*/
        }
    }

    return;
}




/*==============================================================================*/
/* FUNCTION : static singlevectortest_gain_fx () */
/*------------------------------------------------------------------------------*/
/* PURPOSE : Single stage VQ for coding */
/*------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word32*) inp : input gain vector Q18 */
/* _(Word16) dimen : dimension of the input vector */
/* _(Word16) cb_size : codebook size */
/* _(Word16*) weight : Weights for the quanitzation */
/* _(Word32*) codebook : Codebook 19Q13 */
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16*)index :quanitzation index */
/* _(Word32*)recon :Reconstruction 19Q13 */
/*------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _None */
/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*------------------------------------------------------------------------------*/
static void singlevectortest_gain_fx (
    const   Word32* inp, /* i : input gain vector Q18*/
    const   Word16 dimen, /* i : dimension of the input vector */
    const   Word16 cb_size, /* i : codebook size */
    Word16* index, /* o : quanitzation index */
    Word32* recon, /* o : Reconstruction Q18 */
    const   Word32* codebook
    /* i : Codebook Q18*/ )
{
    Word16 k, interNum, flag;
    Word32 meanU, meanQ;
    Word16 least[4];
    Word32 L_tmp;

    interNum = 4;
    move16();

    return_M_Least_fx_GainFrame( inp, codebook, cb_size, interNum, least );

    meanU = sum32_fx( inp, dimen ); /* Q18 */
    Copy32( codebook + dimen * least[0], recon, dimen );

    L_tmp = L_shl( Mult_32_16( meanU, 18022 ), 1 ); /* Q18 */
    index[0] = least[0];
    move16();
    flag = 0;
    move16();
    FOR( k = 0; k < interNum; k++ )
    {
        IF( flag == 0 )
        {
            meanQ = sum32_fx( codebook + dimen * least[k], dimen ); /* Q18 */
            IF( L_sub( meanQ, L_tmp ) <= 0 )
            {
                flag = 1;
                move16();
                Copy32( codebook + dimen * least[k], recon, dimen );
                index[0] = least[k];
                move16();
            }
        }
    }

    return;
}




/*==============================================================================*/
/* FUNCTION : return_M_Least_fx_GainFrame () */
/*------------------------------------------------------------------------------*/
/* PURPOSE : */
/*------------------------------------------------------------------------------*/
/* INPUT ARGUMENTS : */
/* _(Word32*) inp : input Q18 */
/* _(Word16*) weight : input gain weights */
/* _(Word16) n_cols : vector size */
/* _(Word16) num_grp : number of centroids for 1st stage */
/* _(Word16) interNum : number on short list prior to 2nd stage search */
/* _(Word32*) codebook : first stage codebook Q18 */
/*------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS : */
/* _(Word16*)least :return value */
/*------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS : */
/* _None */
/*------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS : */
/* _ None */
/*------------------------------------------------------------------------------*/
static void return_M_Least_fx_GainFrame(
    const Word32* inp,          /* i: input Q18*/
    const Word32* codebook,     /* i: codebook Q18*/
    const Word16 num_grp,       /* i: number of centroids */
    const Word16 interNum,      /* i: number on short list prior to 2nd stage search*/
    Word16* least         /* o: return value */ )
{
    Word16 i, k;
    Word32 distance[1024], mindist;

    Word16 n;
    Word32 diff[1024];
    Word32 max_diff;
    mindist = L_add(MAX_32, 0);

    diff[0] = L_sub( *inp, codebook[0] );
    move32();
    max_diff = L_abs( diff[0] );
    FOR( i = 1; i < num_grp; i++ )
    {
        diff[i] = L_sub( *inp, codebook[i] );
        move32();
        max_diff  = L_max(max_diff , L_abs( diff[i] ));
    }
    n = ( max_diff == 0 )?31:norm_l( max_diff );

    FOR( i = 0; i < num_grp; i++ )
    {
        diff[i] = L_shl( diff[i], n );
        move32();
    }

    FOR( i = 0; i < num_grp; i++ )
    {
        distance[i] = L_deposit_l(0);
        distance[i] = Mult_32_32( diff[i], diff[i] );
        move32();

        if( L_sub( distance[i], mindist ) < 0 )
        {
            least[0] = i;
            move16();
        }
        mindist = L_min(mindist,distance[i]);
    }

    distance[least[0]] = MAX_32;
    move32();

    FOR( k = 1; k < interNum; k++ )
    {
        mindist = L_add(MAX_32, 0);
        FOR( i = 0; i < num_grp; i++ )
        {
            if( L_sub( distance[i], mindist ) < 0 )
            {
                least[k] = i;
                move16();
            }
            mindist = L_min(mindist,distance[i]);
        }

        distance[least[k]] = MAX_32;
        move32();
    }

    return;
}


/*-------------------------------------------------------------------*
* Quant_lower_LSF_fx()
*-------------------------------------------------------------------*/
static void Quant_lower_LSF_fx(
    const   Word16 lsf[],       /* i : Input LSFs           Q15 */
    Word16 lsf_q[],     /* o : Quantized LSFs       Q15 */
    Word16 lsf_idx[]    /* o : Quantized LSFs indices */
)
{
    Word16 i;

    lsf_idx[0] = ( Word16 )squant_fx( lsf[0], &lsf_q[0], lsf_q_cb_fx[0], lsf_q_cb_size_fx[0] );
    FOR( i = 1; i < NUM_Q_LSF; i++ )
    {
        lsf_idx[i] = ( Word16 )squant_fx( sub( lsf[i], lsf_q[i - 1] ), &lsf_q[i], lsf_q_cb_fx[i], lsf_q_cb_size_fx[i] );
        move16();
        lsf_q[i] = add( lsf_q[i - 1], lsf_q[i] );
        move16();
    }

    return;
}


/*-------------------------------------------------------------------*
* Quant_mirror_point_fx()
*-------------------------------------------------------------------*/
static Word16 Quant_mirror_point_fx(
    const   Word16 lsf[],       /* i : Input LSFs */
    const   Word16 lsf_q[],
    Word16* m           /* o : Mirror point */
)
{
    Word16 m_diff;
    Word16 m_idx;

    m_diff = mult_r( sub( lsf[NUM_Q_LSF], lsf_q[NUM_Q_LSF - 1] ), 16384 );

    m_idx = ( Word16 )squant_fx( m_diff, m, mirror_point_q_cb_fx,
                                 MIRROR_POINT_Q_CB_SIZE ); /*move16(); */

    *m = add( lsf_q[NUM_Q_LSF - 1], *m );
    move16();

    return m_idx;
}

/*-------------------------------------------------------------------*
* Find_LSF_grid()
*
* Find the best grid for the LSFs
*-------------------------------------------------------------------*/

static Word16 Find_LSF_grid_fx(
    const   Word16 lsf[],       /* i : Input LSFs */
    Word16 lsf_q[],     /* o : Quantized LSFs */
    const   Word16 m            /* i : Mirror point */
)
{
    Word16 lsf_map[NUM_MAP_LSF];
    Word16 grid[NUM_LSF_GRIDS][NUM_MAP_LSF];
    Word16 offset;
    Word16 last_q_lsf;
    Word16 lsf_t[NUM_MAP_LSF];
    Word16 lsf_smooth[NUM_MAP_LSF];
    Word32 D, D_best;
    Word16 I_best = 0;
    Word16 i, j;
    Word16 scale;

    Word16 tmp, exp, tmp1;
    Word32 L_tmp;


    tmp = shl(m, 1);
    lsf_map[0] = sub( tmp, lsf_q[NUM_MAP_LSF-1-0] );
    move16();
    lsf_map[1] = sub( tmp, lsf_q[NUM_MAP_LSF-1-1] );
    move16();
    lsf_map[2] = sub( tmp, lsf_q[NUM_MAP_LSF-1-2] );
    move16();
    lsf_map[3] = sub( tmp, lsf_q[NUM_MAP_LSF-1-3] );
    move16();
    lsf_map[4] = sub( tmp, lsf_q[NUM_MAP_LSF-1-4] );
    move16();


    IF( sub( m, MAX_LSF_FX_2 ) > 0 )
    {
        offset = lsf_map[0];
        move16();
        exp = norm_s( m );
        tmp = div_s( shl(1, sub(14, exp)), m );
        L_tmp = L_mult( sub( MAX_LSF_FX, m ), tmp );
        scale = round_fx( L_shl( L_tmp, exp + 1 ) );

        FOR( i = 0; i < NUM_MAP_LSF; i++ )
        {
            tmp = mult_r( sub( lsf_map[i], offset ), scale );
            lsf_map[i] = add( tmp, offset );
            move16();
        }
    }

    last_q_lsf = lsf_q[NUM_Q_LSF - 1];
    move16();
    scale = sub( MAX_LSF_FX, last_q_lsf );

    FOR( i = 0; i < NUM_LSF_GRIDS; i++ )
    {
        FOR( j = 0; j < NUM_MAP_LSF; j++ )
        {
            grid[i][j] = add( mult_r( lsf_grid_fx[i][j], scale ), last_q_lsf );
            move16();
        }
    }

    D_best = L_add(MAX_32, 0);
    FOR( i = 0; i < NUM_LSF_GRIDS; i++ )
    {
        D = L_deposit_l(0);
        FOR( j = 0; j < NUM_MAP_LSF; j++ )
        {
            /*lsf_t[j] = (1 - grid_smoothing[j])*lsf_map[j] + grid_smoothing[j]*grid[i][j]; */
            tmp = sub( 32767, grid_smoothing_fx[j] );
            tmp = mult_r( tmp, lsf_map[j] );
            tmp1 = mult_r( grid_smoothing_fx[j], grid[i][j] );
            lsf_t[j] = add( tmp, tmp1 );
            move16();

            tmp = sub( lsf_t[j], lsf[NUM_Q_LSF + j] );
            D = L_mac0( D, tmp, tmp );

            /* D += (lsf_t[j] - lsf[NUM_Q_LSF + j])*(lsf_t[j] - lsf[NUM_Q_LSF + j]); */
        }
        IF( L_sub( D, D_best ) < 0 )
        {
            Copy( lsf_t, lsf_smooth, NUM_MAP_LSF );
            D_best = L_add(D, 0);
            I_best = i;
            move16();
        }
    }

    FOR( i = 0; i < NUM_MAP_LSF; i++ )
    {
        lsf_q[NUM_Q_LSF + i] = lsf_smooth[i];
        move16();
    }

    return I_best;
}

/*-------------------------------------------------------------------*
* gainFrSmooth_En_fx()
*
* Gain frame smoothing and attenuation control
*-------------------------------------------------------------------*/
static void gainFrSmooth_En_fx(Encoder_State_fx *st_fx,
                               Word16 *shb_frame_fx,
                               const Word16 *lpc_shb_fx,
                               const Word16 *lsp_shb_fx,
                               Word16 *MA_lsp_shb_spacing,
                               Word16 *frGainAttenuate,
                               Word16 *frGainSmoothEn
                              )
{
    Word16 temp_shb_frame[L_FRAME16k+L_SHB_LAHEAD];
    Word32 lsp_slow_evol_rate, lsp_fast_evol_rate;
    Word16 lsp_spacing;

    Word32 tempQ31;
    Word16 tempQ15_1, tempQ15_2;
    Word16 i;

    /* inits */
    *frGainAttenuate = 0;
    move16();
    *frGainSmoothEn = 0;
    move16();
    *MA_lsp_shb_spacing = 16384;
    move16();
    lsp_spacing = lsp_shb_fx[0];
    move16();

    /* estimate the mean square error in lsps from current frame to past frames */
    tempQ15_1 = sub( lsp_shb_fx[0], st_fx->lsp_shb_slow_interpl_fx[0] );
    tempQ15_2 = sub( lsp_shb_fx[0], st_fx->lsp_shb_fast_interpl_fx[0] );
    lsp_slow_evol_rate = L_mult(tempQ15_1, tempQ15_1 );
    lsp_fast_evol_rate = L_mult(tempQ15_2, tempQ15_2 );

    /* update the slow and fast lsp interp for next frame */
    tempQ31 = L_mult(st_fx->lsp_shb_slow_interpl_fx[0], 22937 );
    st_fx->lsp_shb_slow_interpl_fx[0] = mac_r(tempQ31, lsp_shb_fx[0], 9830 );
    move16();
    tempQ31 = L_mult(st_fx->lsp_shb_fast_interpl_fx[0], 9830 );
    st_fx->lsp_shb_fast_interpl_fx[0] =  mac_r(tempQ31, lsp_shb_fx[0], 22937 );
    move16();

    FOR( i = 1; i < LPC_SHB_ORDER; i++ )
    {
        tempQ15_1 = sub(lsp_shb_fx[i], lsp_shb_fx[i-1]);
        lsp_spacing = s_min( lsp_spacing, tempQ15_1);

        /* estimate the mean square error in lsps from current frame to past frames */
        tempQ15_1 = sub( lsp_shb_fx[i], st_fx->lsp_shb_slow_interpl_fx[i] );
        tempQ15_2 = sub( lsp_shb_fx[i], st_fx->lsp_shb_fast_interpl_fx[i] );
        lsp_slow_evol_rate = L_mac(lsp_slow_evol_rate, tempQ15_1, tempQ15_1 );
        lsp_fast_evol_rate = L_mac(lsp_fast_evol_rate , tempQ15_2, tempQ15_2 );

        /* update the slow and fast interpolation lsps for next frame */
        tempQ31 = L_mult(st_fx->lsp_shb_slow_interpl_fx[i], 22937 );
        st_fx->lsp_shb_slow_interpl_fx[i] = mac_r(tempQ31, lsp_shb_fx[i], 9830 );
        move16();
        tempQ31 = L_mult(st_fx->lsp_shb_fast_interpl_fx[i], 9830 );
        st_fx->lsp_shb_fast_interpl_fx[i] =  mac_r(tempQ31, lsp_shb_fx[i], 22937 );
        move16();
    }

    test();
    test();
    IF( sub( st_fx->last_extl_fx, SWB_TBE ) != 0
        && sub(st_fx->last_extl_fx, FB_TBE) != 0
        && sub( lsp_spacing, 262 ) < 0 )
    {
        st_fx->lsp_shb_spacing_fx[0] = lsp_spacing;
        move16();
        st_fx->lsp_shb_spacing_fx[1] = lsp_spacing;
        move16();
        st_fx->lsp_shb_spacing_fx[2] = lsp_spacing;
        move16();
        st_fx->prev_frGainAtten_fx = 1;
        move16();
        set16_fx(st_fx->shb_inv_filt_mem_fx, 0, LPC_SHB_ORDER);
    }

    /* Estimate the moving average LSP spacing */
    tempQ31 = L_mult(st_fx->lsp_shb_spacing_fx[0], 3277);         /* 0.1f */
    tempQ31 = L_mac(tempQ31, st_fx->lsp_shb_spacing_fx[1], 6553); /* 0.2f */
    tempQ31 = L_mac(tempQ31, st_fx->lsp_shb_spacing_fx[2], 9830); /* 0.3f */
    *MA_lsp_shb_spacing = mac_r(tempQ31, lsp_spacing, 13107);      /* 0.4f */

    st_fx->lsp_shb_spacing_fx[0] = st_fx->lsp_shb_spacing_fx[1];
    move16();
    st_fx->lsp_shb_spacing_fx[1] = st_fx->lsp_shb_spacing_fx[2];
    move16();
    st_fx->lsp_shb_spacing_fx[2] = lsp_spacing;
    move16();

    test();
    test();
    test();
    IF( ( sub( lsp_spacing, 262 ) < 0 && ( sub( *MA_lsp_shb_spacing, 164 ) < 0 || sub(st_fx->prev_frGainAtten_fx, 1) == 0 ) )
        || sub( lsp_spacing, 105 ) <= 0 )
    {
        *frGainAttenuate = 1;
        move16();

        IF( L_sub(st_fx->total_brate_fx , ACELP_24k40) != 0 )
        {
            Copy( shb_frame_fx, temp_shb_frame, L_FRAME16k + L_SHB_LAHEAD );
            fir_fx( temp_shb_frame, lpc_shb_fx, shb_frame_fx, st_fx->shb_inv_filt_mem_fx, L_FRAME16k + L_SHB_LAHEAD, LPC_SHB_ORDER, 1,3);
        }
        ELSE
        {
            set16_fx(st_fx->shb_inv_filt_mem_fx, 0, LPC_SHB_ORDER);
        }

        test();
        IF ( L_sub( lsp_slow_evol_rate, FL2WORD32(0.001f) ) < 0 &&  L_sub( lsp_fast_evol_rate, FL2WORD32(0.001f) ) < 0 )
        {
            *frGainSmoothEn = 1;
            move16();
        }
    }

}

/*-------------------------------------------------------------------*
* Quant_BWE_LSF()
*
* Quantize super highband spectral envolope
*-------------------------------------------------------------------*/

static void Quant_BWE_LSF_fx(

    Encoder_State_fx* st_fx, /* i/o: encoder state structure */
    const   Word16 lsf_shb[], /* i : unquanitzed LSFs */
    Word16  Q_lsfs[]  /* o : quanitzed LSFs */
)
{
    Word16 lsf[LPC_SHB_ORDER];
    Word16 lsf_q[LPC_SHB_ORDER];
    Word16 lsf_idx[NUM_Q_LSF];
    Word16 i;
    Word16 m_idx;
    Word16 m;
    Word16 grid_idx;

    FOR( i = 0; i < LPC_SHB_ORDER; i++ )
    {
        lsf[i] = sub( 16384, lsf_shb[LPC_SHB_ORDER - 1 - i] );
        move16();
    }

    Quant_lower_LSF_fx( lsf, lsf_q, lsf_idx);
    /* i: lsf in Q15 */
    /* o: lsf_q in Q15 */

    FOR( i = 0; i < NUM_Q_LSF; i++ )
    {
        st_fx->lsf_idx_fx[i] = lsf_idx[i];
        move16();
        IF( sub( st_fx->codec_mode, MODE2 ) != 0 )
        {
            push_indice_fx( st_fx, IND_SHB_LSF, lsf_idx[i], lsf_q_num_bits[i] );
        }
    }

    m_idx = Quant_mirror_point_fx( lsf, lsf_q, &m );

    st_fx->m_idx_fx = m_idx;
    move16();
    IF( sub( st_fx->codec_mode, MODE2 ) != 0 )
    {
        push_indice_fx( st_fx, IND_SHB_MIRROR, m_idx, MIRROR_POINT_BITS );
    }

    grid_idx = Find_LSF_grid_fx( lsf, lsf_q, m );

    st_fx->grid_idx_fx = grid_idx;
    IF( sub( st_fx->codec_mode, MODE2 ) != 0 )
    {
        push_indice_fx( st_fx, IND_SHB_GRID, grid_idx, NUM_LSF_GRID_BITS );
    }

    FOR( i = 0; i < LPC_SHB_ORDER; i++ )
    {
        Q_lsfs[i] = sub( 16384, lsf_q[LPC_SHB_ORDER - 1 - i] );
        move16();
    }

    return;
}

/*-------------------------------------------------------------------*
 * fb_tbe_enc()
 *
 * FB TBE encoder, 14(resp. 15.5) - 20 kHz band encoding module
 *-------------------------------------------------------------------*/

void fb_tbe_enc_fx(
    Encoder_State_fx *st,                /* i/o: encoder state structure                 */
    const Word16 new_input[],        /* i  : input speech at 48 kHz sample rate      */
    const Word16 fb_exc[],           /* i  : FB excitation from the SWB part         */
    Word16 Q_fb_exc
)
{
    Word16 ratio;
    Word16 tmp_vec[L_FRAME48k];
    Word16 idxGain;
    Word16 input_fhb[L_FRAME48k];
    Word16 Sample_Delay_HP;
    Word32 fb_exc_energy, temp2;
    Word32 L_tmp;
    Word16 tmp,tmp1,tmp2,exp,exp2,exp_norm;

    elliptic_bpf_48k_generic_fx( new_input, 0 , tmp_vec, st->elliptic_bpf_2_48k_mem_fx, full_band_bpf_2_fx );

    Sample_Delay_HP = NS2SA(48000, ACELP_LOOK_NS + DELAY_FD_BWE_ENC_12k8_NS + DELAY_FIR_RESAMPL_NS) - L_FRAME48k/2 ;

    IF( sub(st->last_extl_fx,FB_TBE) != 0 )
    {
        set16_fx( st->old_input_fhb_fx, 0, Sample_Delay_HP );
    }

    Copy( st->old_input_fhb_fx, input_fhb, Sample_Delay_HP );
    Copy( tmp_vec, input_fhb + Sample_Delay_HP, L_FRAME48k-Sample_Delay_HP );
    Copy( tmp_vec + L_FRAME48k - Sample_Delay_HP, st->old_input_fhb_fx, Sample_Delay_HP );

    /* Compute the energy of the Fullband component over 4kHz (16kHz to 20kHz) */
    temp2 = sum2_fx( input_fhb, L_FRAME48k/2 );/* Q11 */
    temp2 = L_add(temp2,st->prev_fb_energy_fx);/* Q11 */
    st->prev_fb_energy_fx = sum2_fx( input_fhb + L_FRAME48k/2, L_FRAME48k/2 );/*Q11*/
    fb_exc_energy = sum2_fx( fb_exc, L_FRAME16k );/* Q(2*Q_fb_exc+1) */


    /*ratio = (float) sqrt( temp2 / fb_exc_energy );*/
    L_tmp = L_max(1, temp2); /*Q6*/
    exp = norm_l(L_tmp);
    tmp = extract_h(L_shl(L_tmp, exp));
    exp = sub(sub(31,11), exp);  /* in Q15 (L_tmp in Q6)*/

    exp2 = norm_l(fb_exc_energy);
    tmp2 = extract_h(L_shl(fb_exc_energy, exp2));
    tmp1 = add(add(Q_fb_exc,Q_fb_exc),1);
    exp2 = sub(sub(31,tmp1), exp2);  /* in Q15 (L_tmp in Q6)*/

    exp = sub(exp2, exp); /* Denormalize and substract */
    IF (sub(tmp2, tmp) > 0)
    {
        tmp2 = shr(tmp2, 1);
        exp = add(exp, 1);
    }
    tmp = div_s(tmp2, tmp);
    L_tmp = L_deposit_h(tmp);
    L_tmp = Isqrt_lc(L_tmp, &exp); /*Q(31-exp)*/
    L_tmp = L_max(L_shr(L_tmp,sub(31,exp)),0x1);/* Q0 */

    /* idxGain = (short)( log2_f ((float)ratio) + 0.5f );
       idxGain = max( 0, min(15,idxGain) ); */
    IF(L_sub(L_tmp,32768) >= 0)
    {
        idxGain = 15;
    }
    ELSE
    {
        ratio = extract_l(L_tmp);
        exp_norm = norm_s(ratio);
        idxGain = sub(14, exp_norm);
        idxGain = s_max(0,idxGain);
    }
    /* ratio = (float)(1 << idxGain);*/
    IF( st->codec_mode == MODE2 )
    {
        st->idxGain_fx = idxGain;
    }
    ELSE
    {
        push_indice_fx( st, IND_FB_SLOPE, idxGain, 4 );
    }

    return;
}
void tbe_write_bitstream_fx(
    Encoder_State_fx *st_fx          /* i/o: encoder state structure                 */
)
{
    Word16 i;

    test();
    test();
    test();
    test();
    test();
    IF ( ( st_fx->rf_mode || L_sub( st_fx->total_brate_fx, ACELP_9k60 ) == 0 ) && ( sub( st_fx->bwidth_fx, WB ) == 0 ) )
    {
        /* WB LSF */
        push_next_indice_fx( st_fx, st_fx->lsf_WB_fx, NUM_BITS_LBR_WB_LSF );

        /* WB frame */
        push_next_indice_fx( st_fx, st_fx->gFrame_WB_fx, NUM_BITS_SHB_FrameGain_LBR_WB );
    }
    ELSE IF ( ( L_sub( st_fx->total_brate_fx, ACELP_9k60 ) >= 0 ) && ( L_sub( st_fx->total_brate_fx, ACELP_32k ) <= 0 ) &&
              ( ( sub( st_fx->bwidth_fx, SWB ) == 0 ) || ( sub( st_fx->bwidth_fx, FB ) == 0 ) ) )
    {
        /* LSF coefficients */

        test();
        IF( (sub(st_fx->rf_mode,1)==0) || L_sub( st_fx->total_brate_fx, ACELP_9k60 ) == 0 )
        {
            push_next_indice_fx( st_fx, st_fx->lsf_idx_fx[0], 8 );
        }
        ELSE
        {
            FOR (i = 0; i < NUM_Q_LSF; i++)
            {
                push_next_indice_fx( st_fx, st_fx->lsf_idx_fx[i], lsf_q_num_bits[i] );
            }

            /* LSF mirror points */
            push_next_indice_fx( st_fx, st_fx->m_idx_fx, MIRROR_POINT_BITS );

            /* LSF grid points */
            push_next_indice_fx( st_fx, st_fx->grid_idx_fx, NUM_LSF_GRID_BITS );
        }

        /* Gain shape */
        push_next_indice_fx( st_fx, st_fx->idxSubGains_fx, NUM_BITS_SHB_SUBGAINS );

        /* frame gain */
        push_next_indice_fx( st_fx, st_fx->idxFrameGain_fx, NUM_BITS_SHB_FRAMEGAIN );

        IF ( L_sub( st_fx->total_brate_fx, ACELP_24k40 ) >= 0 )
        {
            /* sub frame energy*/
            push_next_indice_fx( st_fx, st_fx->idx_shb_fr_gain_fx, NUM_BITS_SHB_ENER_SF );

            /* gain shapes residual */
            FOR (i = 0; i < NB_SUBFR16k; i++)
            {
                push_next_indice_fx( st_fx, st_fx->idx_res_gs_fx[i], NUM_BITS_SHB_RES_GS );
            }

            /* voicing factor */
            push_next_indice_fx( st_fx, st_fx->idx_mixFac_fx, NUM_BITS_SHB_VF );
        }

        IF( sub(st_fx->tec_tfa, 1) == 0 )
        {
            push_next_indice_fx( st_fx, st_fx->tec_flag, BITS_TEC );
            push_next_indice_fx( st_fx, st_fx->tfa_flag, BITS_TFA );
        }
    }

    IF ( sub( st_fx->bwidth_fx, FB ) == 0 )
    {
        push_next_indice_fx( st_fx, st_fx->idxGain_fx, 4 );
    }
}


void TBEreset_enc_fx(
    Encoder_State_fx *st_fx,          /* i/o: encoder state structure                 */
    Word16 bandwidth                  /* i  : bandwidth mode                          */
)
{
    set16_fx( st_fx->old_bwe_exc_fx, 0, PIT16k_MAX * 2 );
    st_fx->bwe_non_lin_prev_scale_fx = L_deposit_l(0);
    st_fx->prev_Q_bwe_exc = 31;
    move16();

    test();
    IF( sub( bandwidth, WB ) == 0 )
    {
        wb_tbe_extras_reset_fx( st_fx->mem_genSHBexc_filt_down_wb2_fx, st_fx->mem_genSHBexc_filt_down_wb3_fx );

        set16_fx( st_fx->state_syn_shbexc_fx, 0, L_SHB_LAHEAD/4 );
        set16_fx( st_fx->syn_overlap_fx, 0, L_SHB_LAHEAD );
        set32_fx( st_fx->mem_csfilt_fx, 0, 2 );
    }
    ELSE IF( ( sub( bandwidth, SWB ) == 0 ) || ( sub( bandwidth, FB ) == 0 ) )
    {
        set16_fx( st_fx->state_ana_filt_shb_fx, 0, (2*ALLPASSSECTIONS_STEEP+1) );

        swb_tbe_reset_fx( st_fx->mem_csfilt_fx, st_fx->mem_genSHBexc_filt_down_shb_fx, st_fx->state_lpc_syn_fx,
                          st_fx->syn_overlap_fx, st_fx->state_syn_shbexc_fx, &(st_fx->tbe_demph_fx), &(st_fx->tbe_premph_fx),
                          st_fx->mem_stp_swb_fx, &(st_fx->gain_prec_swb_fx) );


        IF( sub( bandwidth, FB ) == 0 )
        {
            fb_tbe_reset_enc_fx( st_fx->elliptic_bpf_2_48k_mem_fx, &(st_fx->prev_fb_energy_fx) );
        }
    }
}


