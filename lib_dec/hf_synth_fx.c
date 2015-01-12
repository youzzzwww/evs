/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"
#include "basop32.h"
/*---------------------------------------------------------------------*
 * Local constants
 *---------------------------------------------------------------------*/

#define L_FIR           31
#define L_SUBFR48k      240
#define L_SUBFR32k      160
/*---------------------------------------------------------------------*
 * Local functions
 *---------------------------------------------------------------------*/

static void filt_6k_7k_scale_fx(Word16 signal[], Word16 lg, Word16 mem[], Word16 fact, Word16 exp);
static void hf_synthesis_fx( const Word32  core_brate, const Word16 output_subfr,const Word16 Aq[],
                             const Word16 exc[], const Word16 Q_exc,Word16 synth[],Word16 synth16k[],Word16 *seed2, Word16 *mem_hp400,Word16 *mem_syn_hf,
                             Word16 *mem_hf, const Word16 Q_syn,Word16 *delay_syn_hf,Word16 *memExp1,Word16 *mem_hp_interp, const Word16 extl, const Word16 CNG_mode );


static void hf_synthesis_amr_wb_fx( const Word32 core_brate, const Word16 output_subfr, const Word16 Ap[], Word16 exc16k[],
                                    Word16 synth_out[], Word16 *mem_syn_hf, Word16 *delay_syn_hf, Word16 *mem_hp_interp, Word16 p_r,
                                    Word16 HF_corr_gain, Word16 til, Word16 voice_factors, const Word16 exc[], const Word16 Q_exc, const Word16 Q_out, Word16 qhf );
static void envelope_fx( const Word32  core_brate, const Word16 Aq[], Word16 Ap[], Word16 *r, Word16 tilt0, Word16 tilt,
                         Word16 voice_factor, Word16 *prev_r, Word16 *voice_fac, Word16 *unvoicing, Word16 *unvoicing_sm, Word16 *unvoicing_flag);
static void AdaptiveStartBand_fx( Word16 *start_band, const Word32  rate, const Word16 *lsf, const Word16 voicing_fac, const Word16 clas, Word16 *voicing_flag,
                                  Word16 *start_band_old, Word32 *OptCrit_old );



/*-------------------------------------------------------------------*
 * hf_synth_init()
 *
 * hf synthesis filters initialization
 * - initialization of 400 Hz high pass filter
 * - initialization of band pass 6kHz to 7kHz FIR filter
 *-------------------------------------------------------------------*/
void hf_synth_init_fx(
    Word16 mem_hp400[],  /* o  : 400 Hz high pass filter memory initialization    */
    Word16 mem_hf[]      /* o  : band pass 6kHz to 7kHz FIR filter initialization */
)
{
    Word16 i;

    FOR (i=0; i<6; i++)
    {
        mem_hp400[i] = 0;
        move16();
    }

    FOR(i=0; i<2*L_FILT16k; i++)
    {
        mem_hf[i] = 0;
        move16();
    }

    return;
}
void hf_synth_reset_fx(
    Word16 *seed2,             /* i/o: random seed for HF noise gen    */
    Word16 mem_hf[],           /* o  : HF band-pass filter memory      */
    Word16 mem_syn_hf[],       /* o  : HF synthesis memory             */
    Word16 mem_hp400[],        /* o  : memory of hp 400 Hz filter      */
    Word16 mem_hp_interp[],    /* o  : interpol. memory                */
    Word16 delay_syn_hf[]      /* o  : HF synthesis memory             */
)
{
    Word16 i;

    FOR( i=0; i<L_FRAME16k; i++ )
    {
        Random( seed2 );
    }

    set16_fx( mem_hf, 0, 2*L_FILT16k );
    set16_fx( mem_syn_hf, 0, M16k );
    set16_fx( mem_hp400, 0, 4 );
    set16_fx( delay_syn_hf, 0, NS2SA(16000,DELAY_CLDFB_NS) );
    set16_fx( mem_hp_interp, 0, 2*L_FILT16k );

    return;
}

/*---------------------------------------------------------------------*
 * hf_synth()
 *
 * High frequency regeneration
 *---------------------------------------------------------------------*/

void hf_synth_fx(
    const Word32  core_brate,		/* i  : core bitrate                   */
    const Word16 output_frame,	/* i  : output frame length            */
    const Word16 *Aq,				/* i  : quantized Az                   */
    const Word16 *exc,         	/* i  : excitation at 12.8 kHz         */
    Word16 *synth,        	/* i  : 12.8kHz synthesis signal       */
    Word16 *synth16k,     	/* o  : 16kHz synthesis signal         */
    Word16 *seed2,        	/* i/o: random seed for HF noise gen   */
    Word16 *mem_hp400,    	/* i/o: memory of hp 400 Hz filter     */
    Word16 *mem_syn_hf,   	/* i/o: HF synthesis memory            */
    Word16 *mem_hf,       	/* i/o: HF band-pass filter memory     */
    const Word16 Q_exc,         	/* i  : excitation scaling             */
    const Word16 Q_syn2,        	/* i  : synthesis scaling              */
    Word16 *delay_syn_hf,	/*i/o: HF synthesis memory             */
    Word16 *memExp1,	    /* o  : HF excitation exponent         */
    Word16 *mem_hp_interp,  /* i/o: interpol. memory               */
    const Word16 extl,              /* i  : flag indicating BWE            */
    const Word16 CNG_mode           /* i  : CNG_mode                       */
)
{
    const Word16 *p_Aq;
    Word16 i_subfr, output_subfr;

    output_subfr = output_frame/NB_SUBFR;
    move16();

    p_Aq = Aq;
    move16();
    FOR( i_subfr=0; i_subfr<L_FRAME; i_subfr+=L_SUBFR )
    {
        hf_synthesis_fx( core_brate, output_subfr, p_Aq, &exc[i_subfr], Q_exc, &synth[i_subfr], &synth16k[i_subfr * output_subfr/L_SUBFR],
                         seed2, mem_hp400, mem_syn_hf, mem_hf,
                         Q_syn2, delay_syn_hf, memExp1, mem_hp_interp, extl, CNG_mode );

        p_Aq  += (M+1);
        move16();
    }

    return;
}
/*-----------------------------------------------------------------------------------*
 * hf_synthesis()
 *
 * HF noise synthesis
 * - Generate HF noise between 6 and 7 kHz.
 * - Set energy of noise according to synthesis tilt.
 *     tilt > 0.8 ==> - 14 dB (voiced)
 *     tilt   0.5 ==> - 6 dB  (voiced or noise)
 *     tilt < 0.0 ==>   0 dB  (noise)
 *-----------------------------------------------------------------------------------*/

static void hf_synthesis_fx(
    const Word32 core_brate,    /* i  : core bitrate                                */
    const Word16 output_subfr,   /* i  : output sub-frame length                     */
    const Word16 Aq[],           /* i  : quantized Az                  Q12  		 */
    const Word16 exc[],          /* i  : excitation at 12.8 kHz      Q_exc     		 */
    const Word16 Q_exc,          /* i  : excitation scaling                 		 */
    Word16 synth[],        /* i  : 12.8kHz synthesis signal    Q_syn     		 */
    Word16 synth16k[],     /* i/o: 16kHz synthesis signal        Q_syn		 */
    Word16 *seed2,         /* i/o: random seed for HF noise gen       		 */
    Word16 *mem_hp400,     /* i/o: memory of hp 400 Hz filter         		 */
    Word16 *mem_syn_hf,    /* i/o: HF synthesis memory                		 */
    Word16 *mem_hf,        /* i/o: HF band-pass filter memory          		 */
    const Word16 Q_syn,          /* i  : synthesis scaling                   		 */
    Word16 *delay_syn_hf,  /* i/o: HF synthesis memory	 Q_syn  			 */
    Word16 *memExp1,       /* o  : HF excitation scaling exponent				 */
    Word16 *mem_hp_interp, /* i/o: interpol. memory                            */
    const Word16 extl,           /* i  : flag indicating BWE                         */
    const Word16 CNG_mode        /* i  : CNG_mode                                    */
)
{
    Word16 i;
    Word16 HF_syn[L_SUBFR16k], upsampled_HF_syn[L_FRAME48k/NB_SUBFR];
    Word16 HF_exc[L_SUBFR16k];
    Word16 temp_buffer[NS2SA(16000,DELAY_CLDFB_NS) - L_FILT16k];
    Word16 tmp, ener, exp1, exp2, scale, delay;
    Word32 L_tmp;
    Word16 Ap[M16k+1];
    (void)extl;
    (void)CNG_mode;

    /*-----------------------------------------------------------------*
     * generate white noise vector
     *-----------------------------------------------------------------*/

    Random_Fill(seed2, L_SUBFR16k, HF_exc, 3); /* 3 = Shift Right by 3 */

    /*-----------------------------------------------------------------*
     * calculate energy scaling factor so that white noise would have the
     * same energy as exc12k8
     *-----------------------------------------------------------------*/

    /*ener = sum2_f( exc, L_SUBFR ) + 0.01f*/
    ener = extract_h(Dot_product12(exc, exc, L_SUBFR, &exp2));
    exp2 = sub(exp2, add(Q_exc, Q_exc));

    /*tmp = round_fx(Dot_product12(HF_exc, HF_exc, output_subfr, &exp1)); */
    L_tmp = Dot_product12(HF_exc, HF_exc, L_SUBFR16k, &exp1);
    tmp = round_fx(L_tmp);
    /* tmp = (float)(sqrt(ener/tmp)) */
    /* scale is -1 if tmp > ener */
    scale = shr(sub(ener, tmp), 15);
    tmp = shl(tmp, scale);
    exp1 = sub(exp1, scale);

    tmp = div_s(tmp, ener);
    exp1 = sub(exp1, exp2);

    L_tmp = L_deposit_h(tmp);

    L_tmp = Isqrt_lc(L_tmp, &exp1);
    scale = round_fx(L_tmp);

    exp2 = sub(*memExp1, exp1);
    move16();
    *memExp1 = exp1;
    move16();

    /*-----------------------------------------------------------------*
     * calculate energy scaling factor to respect tilt of synth12k8
     * (tilt: 1=voiced, -1=unvoiced)
     *-----------------------------------------------------------------*/

    hp400_12k8_fx( synth, L_SUBFR, mem_hp400 );
    L_tmp = L_mac(1L, synth[0], synth[0]);
    FOR (i = 1; i < L_SUBFR; i++)
    {
        L_tmp = L_mac(L_tmp, synth[i], synth[i]);
    }
    tmp = norm_l(L_tmp);
    ener = extract_h(L_shl(L_tmp, tmp));   /* ener = r[0] */

    L_tmp = L_mac(1L, synth[1], synth[0]);
    FOR (i = 2; i < L_SUBFR; i++)
    {
        L_tmp = L_mac(L_tmp, synth[i], synth[i - 1]);
    }
    tmp = extract_h(L_shl(L_tmp, tmp));    /* tmp = r[1] */
    tmp = s_max(0, tmp);
    if (tmp > 0)
    {
        tmp = div_s(tmp, ener);
    }

    /*-----------------------------------------------------------------*
     * modify energy of white noise according to synthesis tilt
     *-----------------------------------------------------------------*/

    /* tmp = 1.0 - fac */
    tmp = add(1, sub(32767, tmp));

    test();
    if( L_sub(core_brate,FRAME_NO_DATA) == 0 || L_sub(core_brate,SID_2k40) == 0)
    {
        /* emphasize HF noise in CNG */
        /*fac *= 2.0f;*/
        tmp = add(tmp, tmp);
    }
    tmp = s_max(tmp, 3277); /* 0.1 *//*Q15 */

    /*scale *= fac;*/
    tmp  = mult_r(scale, tmp);
    /*-----------------------------------------------------------------*
     * modify HF excitation according to both calculated scaling factors
     * high pass filtering (0.94ms of delay)
     *-----------------------------------------------------------------*/

    filt_6k_7k_scale_fx( HF_exc, L_SUBFR16k, mem_hf,tmp,exp2 );

    /*-----------------------------------------------------------------*
     * synthesis of noise: 4.8kHz..5.6kHz --> 6kHz..7kHz
     *-----------------------------------------------------------------*/

    /*weight_a( Aq, Ap, 0.6f, M );*/
    weight_a_lc_fx(Aq, Ap, Gamma_19661_Tbl_fx, M);
    Syn_filt_s( 2, Ap, M, HF_exc, HF_syn, L_SUBFR16k, mem_syn_hf + (M16k - M), 1 );
    Scale_sig(HF_syn, L_SUBFR16k, (add(Q_syn, exp1))+2);
    Scale_sig(mem_syn_hf + (M16k - M), M, (add(Q_syn, exp1))+2);

    /*-----------------------------------------------------------------*
     * add filtered HF noise to speech synthesis
     *-----------------------------------------------------------------*/

    /* delay by 5 samples @16kHz to compensate CLDFB resampling delay (20samples) and HP filtering delay (roughly 15 samples) */
    delay = NS2SA(16000,DELAY_CLDFB_NS) - 15;
    Copy( HF_syn+L_SUBFR16k-delay, temp_buffer, delay );
    Copy( HF_syn, HF_syn+delay, L_SUBFR16k-delay );
    Copy( delay_syn_hf, HF_syn, delay );
    Copy( temp_buffer, delay_syn_hf, delay );

    /* interpolate the HF synthesis */
    IF( sub(output_subfr,L_SUBFR48k) == 0 )   /* 48kHz sampled output */
    {
        {
            Word16 s;
            s = s_max
                (
                    s_min
                    (
                        sub(s_min(Find_Max_Norm16(HF_syn, L_SUBFR16k), Find_Max_Norm16(mem_hp_interp, INTERP_3_1_MEM_LEN - 3)), 3),
                        sub(Find_Max_Norm16(mem_hp_interp + INTERP_3_1_MEM_LEN - 3, 3), 1)
                    ),
                    0
                );
            Scale_sig(HF_syn, L_SUBFR16k, s);
            Scale_sig(mem_hp_interp, INTERP_3_1_MEM_LEN, s);
            interpolate_3_over_1_allpass_fx( HF_syn, L_SUBFR16k, upsampled_HF_syn, mem_hp_interp, allpass_poles_3_ov_2 );
            Scale_sig(upsampled_HF_syn, 3*L_SUBFR16k, -s);
            Scale_sig(mem_hp_interp, INTERP_3_1_MEM_LEN, -s);
            Scale_sig(HF_syn, L_SUBFR16k, -s);
        }
        Scale_sig( upsampled_HF_syn, L_SUBFR48k, -1 );
    }
    ELSE IF( sub(output_subfr,L_SUBFR32k) == 0 ) /* 32kHz sampled output */
    {
        {
            Word16 s;
            s = s_max(sub(s_min(Find_Max_Norm16(HF_syn, L_SUBFR16k), Find_Max_Norm16(mem_hp_interp, 2*ALLPASSSECTIONS_STEEP)),2), 0);
            Scale_sig(HF_syn, L_SUBFR16k, s);
            Scale_sig(mem_hp_interp, 2*ALLPASSSECTIONS_STEEP, s);
            Interpolate_allpass_steep_fx( HF_syn, mem_hp_interp, L_SUBFR16k, upsampled_HF_syn );
            Scale_sig(upsampled_HF_syn, 2*L_SUBFR16k, -s);
            Scale_sig(mem_hp_interp, 2*ALLPASSSECTIONS_STEEP, -s);
            Scale_sig(HF_syn, L_SUBFR16k, -s);
        }
    }
    ELSE    /* 16kHz sampled output */
    {
        Copy( HF_syn, upsampled_HF_syn, L_SUBFR16k );
    }

    Vr_add( synth16k, upsampled_HF_syn, synth16k, output_subfr );

    return;
}


/*-------------------------------------------------------------------*
 * filt_6k_7k:
 *
 * 15th order band pass 6kHz to 7kHz FIR filter.
 *
 * frequency:  4kHz   5kHz  5.5kHz  6kHz  6.5kHz 7kHz  7.5kHz  8kHz
 * dB loss:   -60dB  -45dB  -13dB   -3dB   0dB   -3dB  -13dB  -45dB
 * (gain=4.0)
 *-------------------------------------------------------------------*/
static void filt_6k_7k_scale_fx(
    Word16 signal[],   /* i/o: signal           */
    Word16 lg,         /* i  : length of input  */
    Word16 mem[],      /* i/o: memory (size=30) */
    Word16 fact,       /* i  : multiply factor  */
    Word16 exp         /* i  : Mem Exponent     */
)
{
    Word16 i, x[L_FRAME48k/NB_SUBFR+(L_FIR-1)];
    Word32 L_tmp;

    Copy_Scale_sig(mem, x, L_FIR - 1, exp);

    FOR (i = 0; i < lg; i++)
    {
        x[i + L_FIR - 1] = shr(mult(signal[i], fact), 2);
        move16();   /* gain of filter = 4 */
    }
    FOR (i = 0; i < lg; i++)
    {
        L_tmp = L_mult(x[i], fir_6k_7k_fx[0]);

        /* Inner Loop Unrolled */
        /* All fir_6k_7k_fx[j] could be replaced by Constants and */
        /* thus the table could be removed from rom_com_fx */
        L_tmp = L_mac(L_tmp, x[i+1], fir_6k_7k_fx[1]);
        L_tmp = L_mac(L_tmp, x[i+2], fir_6k_7k_fx[2]);
        L_tmp = L_mac(L_tmp, x[i+3], fir_6k_7k_fx[3]);
        L_tmp = L_mac(L_tmp, x[i+4], fir_6k_7k_fx[4]);
        L_tmp = L_mac(L_tmp, x[i+5], fir_6k_7k_fx[5]);
        L_tmp = L_mac(L_tmp, x[i+6], fir_6k_7k_fx[6]);
        /*L_tmp = L_mac(L_tmp, x[i+7], fir_6k_7k_fx[7]); Coef is 0 */
        L_tmp = L_mac(L_tmp, x[i+8], fir_6k_7k_fx[8]);
        L_tmp = L_mac(L_tmp, x[i+9], fir_6k_7k_fx[9]);
        L_tmp = L_mac(L_tmp, x[i+10], fir_6k_7k_fx[10]);
        L_tmp = L_mac(L_tmp, x[i+11], fir_6k_7k_fx[11]);
        L_tmp = L_mac(L_tmp, x[i+12], fir_6k_7k_fx[12]);
        L_tmp = L_mac(L_tmp, x[i+13], fir_6k_7k_fx[13]);
        L_tmp = L_mac(L_tmp, x[i+14], fir_6k_7k_fx[14]);
        L_tmp = L_mac(L_tmp, x[i+15], fir_6k_7k_fx[15]);
        L_tmp = L_mac(L_tmp, x[i+16], fir_6k_7k_fx[16]);
        L_tmp = L_mac(L_tmp, x[i+17], fir_6k_7k_fx[17]);
        L_tmp = L_mac(L_tmp, x[i+18], fir_6k_7k_fx[18]);
        L_tmp = L_mac(L_tmp, x[i+19], fir_6k_7k_fx[19]);
        L_tmp = L_mac(L_tmp, x[i+20], fir_6k_7k_fx[20]);
        L_tmp = L_mac(L_tmp, x[i+21], fir_6k_7k_fx[21]);
        L_tmp = L_mac(L_tmp, x[i+22], fir_6k_7k_fx[22]);
        /*L_tmp = L_mac(L_tmp, x[i+23], fir_6k_7k_fx[23]); Coef is 0 */
        L_tmp = L_mac(L_tmp, x[i+24], fir_6k_7k_fx[24]);
        L_tmp = L_mac(L_tmp, x[i+25], fir_6k_7k_fx[25]);
        L_tmp = L_mac(L_tmp, x[i+26], fir_6k_7k_fx[26]);
        L_tmp = L_mac(L_tmp, x[i+27], fir_6k_7k_fx[27]);
        L_tmp = L_mac(L_tmp, x[i+28], fir_6k_7k_fx[28]);
        L_tmp = L_mac(L_tmp, x[i+29], fir_6k_7k_fx[29]);
        L_tmp = L_mac(L_tmp, x[i+30], fir_6k_7k_fx[30]);

        signal[i] = round_fx(L_tmp);
    }
    Copy(x + lg, mem, L_FIR - 1);
}

/*-------------------------------------------------------------------*
 * hf_synth_amr_wb_init()
 *
 * hf synthesis filters initialization
 * - initialization of 1600 Hz low pass filter
 * - initialization of band pass 6kHz to 8kHz FIR filter for noise and line resampled signals
 *-------------------------------------------------------------------*/

void hf_synth_amr_wb_init_fx(
    Word16 *prev_r,                    /* o  : previous sub-frame gain                  */
    Word16 *fmerit_w_sm,               /* o  : 1 sample memory fmerit_w param*/
    Word16 *frame_count,               /* o  : frame counter initialization                     */
    Word16 *ne_min,                    /* o  : minimum Noise gate - short-term energy initialization*/
    Word16 *fmerit_m_sm,               /* o  : 1 sample memory fmerit_m param                   */
    Word16 *voice_fac,                 /* o  : voice factor initialization                      */
    Word16 *unvoicing,                 /* o  : unvoiced parameter                               */
    Word16 *unvoicing_sm,              /* o  : smoothed unvoiced parameter                      */
    Word16 *unvoicing_flag,            /* o  : unvoiced flag                                    */
    Word16 *voicing_flag,              /* o  : voiced flag                                      */
    Word16 *start_band_old,            /* o  : previous start point for copying frequency band  */
    Word32 *OptCrit_old                /* o  : previous criterion value for deciding the start point */
)
{
    *prev_r = 0;
    move16();
    *fmerit_w_sm = 0;
    move16();
    *frame_count = 0;
    move16();
    *ne_min = -7680;
    move16(); /*Q8*/
    *fmerit_m_sm = 0;
    move16();
    *voice_fac = 0;
    move16();
    *unvoicing = 0;
    move16();
    *unvoicing_sm = 32767;
    move16(); /*Q15*/
    *unvoicing_flag = 0;
    move16();
    *voicing_flag = 0;
    move16();
    *start_band_old = 160;
    move16();
    *OptCrit_old = 32768;
    move32(); /*Q15*/ ;
    return;
}

/*-------------------------------------------------------------------*
 * hf_synth_amr_wb_reset()
 *
 * reset of HF synthesis filters
 * - needed in switching scenarios
 *-------------------------------------------------------------------*/

void hf_synth_amr_wb_reset_fx(
    Word16 *seed2,                     /* i/o: random seed for HF noise gen                      */
    Word16 mem_syn_hf[],               /* o  : HF synthesis memory                               */
    Word16 mem_hp_interp[],            /* o  : interpol. memory                                  */
    Word16 *prev_r,                    /* o  : previous sub-frame gain                  */
    Word16 *fmerit_w_sm,               /* o  : 1 sample memory fmerit_w param                    */
    Word16 delay_syn_hf[],             /* o  : HF synthesis memory                               */
    Word16 *frame_count,               /* o  : frame counter memory                              */
    Word16 *ne_min,                    /* o  : minimum Noise gate - short-term energy memory     */
    Word16 *fmerit_m_sm,               /* o  : 1 sample memory fmerit_m param                    */
    Word16 *voice_fac,                 /* o  : voice factor memory                               */
    Word16 *unvoicing,                 /* o  : unvoiced parameter                               */
    Word16 *unvoicing_sm,              /* o  : smoothed unvoiced parameter                      */
    Word16 *unvoicing_flag,            /* o  : unvoiced flag                                    */
    Word16 *voicing_flag,              /* o  : voiced flag                                      */
    Word16 *start_band_old,            /* o  : previous start point for copying frequency band  */
    Word32 *OptCrit_old                /* o  : previous criterion value for deciding the start point */
)
{
    Word16 i;


    FOR( i=0; i<L_FRAME16k; i++ )
    {
        Random( seed2 );
    }
    set16_fx( mem_syn_hf, 0, M16k);
    set16_fx( delay_syn_hf, 0, NS2SA(16000,DELAY_CLDFB_NS) );
    set16_fx( mem_hp_interp, 0, 2*L_FILT16k);

    *prev_r = 0;
    move16();
    *fmerit_w_sm = 0;
    move16();
    *frame_count = 0;
    move16();
    *ne_min = -7680;
    move16(); /*Q8*/
    *fmerit_m_sm = 0;
    move16();
    *voice_fac = 0;
    move16();
    *unvoicing = 0;
    move16();
    *unvoicing_sm = 32767;
    move16(); /*Q15*/
    *unvoicing_flag = 0;
    move16();
    *voicing_flag = 0;
    move16();
    *start_band_old = 160;
    move16();
    *OptCrit_old = 32768;
    move32(); /*Q15*/

    return;
}

/*-------------------------------------------------------------------*
 * hf_synth_amr_wb()
 *
 * HF synthesis in AMR-WB IO
 *-------------------------------------------------------------------*/

void hf_synth_amr_wb_fx(
    const Word32 core_brate,               /* i  : core bitrate                                      */
    const Word16 output_frame,              /* i  : output frame length                               */
    const Word16 *Aq,                       /* i  : quantized Az : Q12                                */
    const Word16 *exc,                      /* i  : excitation at 12.8 kHz  : Q_exc                    */
    Word16 *synth,                    /* i/o: synthesis signal at 12.8k : Q_syn                  */
    Word16 *mem_syn_hf,               /* i/o: HF synthesis memory : Q_out                        */
    Word16 *delay_syn_hf,             /* i/o: HF synthesis memory  : Q_out                       */
    Word16 *prev_r,                   /* i/o  : previous sub-frame gain  : Q10                  */
    Word16 *fmerit_w_sm,              /* i/o: smoothed fmerit_w    : Q14                        */
    Word16 *amr_io_class,             /* i  : signal class (determined by FEC algorithm)        */
    Word16 *mem_hp_interp,            /* i/o: interpol. memory : Q_out                           */
    Word16 *synth_out,                /* i/o: output signal at output Fs  : Q_out                */
    Word16 fmerit,                    /* i  : classify parameter from FEC : Q14                 */
    const Word16 *hf_gain,                  /* i  : decoded HF gain                                   */
    const Word16 *voice_factors,            /* i  : voicing factors  : Q15                            */
    const Word16 pitch_buf[],               /* i  : pitch buffer  : Q5                                */
    const Word16 ng_ener_ST,                /* i  : Noise gate - short-term energy : Q8               */
    const Word16 *lsf_new,                  /* i  : ISF vector  : Q2                                  */
    Word16 *frame_count,              /* i/o: frame counter                                     */
    Word16 *ne_min,                   /* i/o: minimum Noise gate  : Q8                          */
    Word16 *fmerit_m_sm,              /* i/o: smoothed fmerit_m   : Q14                         */
    Word16 *voice_facor_sm,           /* o  : voice factor memory : Q15                         */
    Word16 *unvoicing,                /* o  : unvoiced parameter  : Q15                         */
    Word16 *unvoicing_sm,             /* o  : smoothed unvoiced parameter  : Q15                */
    Word16 *unvoicing_flag,           /* o  : unvoiced flag                                     */
    Word16 *voicing_flag,             /* o  : voiced flag                                       */
    Word16 *start_band_old,           /* o  : previous start point for copying frequency band   */
    Word32 *OptCrit_old,              /* o  : previous criterion value for deciding the start point */
    const Word16 Q_exc,                      /* i  : exc scaling                                       */
    const Word16 Q_out                       /* i  : Q_syn2-1                                          */
)
{
    Word16 core_type = 1;
    Word16 start_band;
    Word16 HF_corr_gain[NB_SUBFR];
    Word16 filt_weight_coeff;
    Word16 i, j, output_subfr;
    Word16 *pt3, *pt4, *pt5;
    const Word16 *pt6, *pt7;
    Word16 *pt1, *pt2;
    Word32 enr1, enr2;
    Word16 til[NB_SUBFR], til0[NB_SUBFR];
    Word16 tmp1, tmp2;
    Word32 L_tmp;
    Word16 exp;
    Word16 pitch_var_cur;
    Word16 voice_fac;
    Word16 fmerit_w;
    Word16 fmerit_m;
    Word16 *p_Ap;
    const Word16 *p_Aq;
    Word16 Ap[(M16k+1)*NB_SUBFR];
    Word16 sub_gain[NB_SUBFR];
    Word16 exc16k[L_FRAME16k] ;
    Word16 qhf;
    Word16 dct_exc[L_FRAME], dct_hb[L_FRAME16k], filt_weight[L_SUBFR16k];
    Word16 hb_ener, g, hb_tonal[L_SUBFR16k], tonal_ener, hb_amb[L_SUBFR16k], inv_g;
    Word16 fb, fn, signum[L_SUBFR16k];
    Word16 alpha, beta, ener, tmp, scale, rev_filt_weight_coeff, qdct, q1, q2, q3, q4, shift;
    Word16 e_subfr1, e_subfr2;
    Word32 exc32[L_FRAME], dct_exc32[L_FRAME], dct_hb32[L_FRAME16k], exc16k32[L_FRAME16k];
    Word16 q_tmp;

    Word16 gamma;

    Scale_sig(synth, L_FRAME, -3);

    pt1 = synth+1;
    pt2 = synth;
    pt3 = til;
    pt4 = til0;
    FOR( i=0; i<NB_SUBFR; i++ )
    {
        enr1 = Dot_product(pt2, pt2, L_SUBFR);      /*2*(Q_syn-3)+1 */
        enr2 = Dot_product(pt1, pt2, L_SUBFR-1);    /*2*(Q_syn-3)+1 */

        tmp1 = extract_h(enr1);
        tmp1 = s_max(1, tmp1);
        tmp2 = extract_h(enr2);

        exp = norm_s(tmp1);
        tmp1 = div_s(shl(1,sub(14,exp)), tmp1);            /*Q(29-exp-2*(Q_syn-3)-1) */
        L_tmp = L_mult(tmp2, tmp1);                 /*30-exp-2*(Q_syn-3)-1+2*(Q_syn-3)+1 - >30-exp */
        *pt3 = round_fx(L_shl(L_tmp, sub(exp,1)));  /*13 */
        *pt4++ = *pt3++;
        move16();

        pt1 += L_SUBFR;
        pt2 += L_SUBFR;
    }

    output_subfr = shr(output_frame , 2);

    if( sub(*amr_io_class, 7) != 0 )
    {
        core_type = 0;
        move16();
    }

    /* modify LF parameters for excitation weighting or sub-frame gains calculating */
    pitch_var_cur = 0;
    move16();
    pt6 = pitch_buf;
    pt7 = pitch_buf+1;
    FOR( i = 0; i < 3; i++ )
    {
        tmp1 = abs_s(sub(*pt6++, *pt7++));  /*Q5 */
        pitch_var_cur = add(pitch_var_cur, shr(tmp1,1)); /*Q6 -> Q5 */
    }
    test();
    IF( sub(*frame_count, FRAME_COUNT) > 0 && *amr_io_class == UNVOICED_CLAS )
    {
        *frame_count = 0;
        move16();
        *ne_min = -7680;
        move16();/*Q8;  */
    }
    ELSE
    {
        tmp1 = *frame_count;
        move16();
        *frame_count = add(*frame_count ,1);
        if(sub(tmp1, 2*FRAME_COUNT)>0)
        {
            *frame_count = 2*FRAME_COUNT;
            move16();
        }
        if ( sub(ng_ener_ST, *ne_min) < 0 )
        {
            *ne_min = ng_ener_ST;
            move16();/*Q8; */
        }
    }

    pt6 = voice_factors;
    L_tmp =  L_mult(*pt6++,4096);
    FOR(i = 1; i < 4; i++)
    {
        L_tmp =  L_mac(L_tmp, *pt6++,4096);
    }

    voice_fac = round_fx(L_tmp);

    /*fmerit_w = fmerit > 5734 ? 5734 : (fmerit < 2458 ? 2458 : fmerit); //Q14 */

    fmerit_w = s_min(fmerit,5734);
    fmerit_w = s_max(fmerit_w,2458);

    if ( sub(core_type, 1) == 0 )
    {
        fmerit_w = shr(fmerit_w, 1); /*Q14; */
    }

    L_tmp = L_mult(fmerit_w, add(16384, voice_fac));
    fmerit_w = extract_l(L_shr(L_tmp, 15)); /*Q14 */
    /**fmerit_w_sm = add(mult_r(*fmerit_w_sm, 29491), mult_r(fmerit_w, 3277));  //Q14 */
    *fmerit_w_sm = round_fx(L_mac(L_mult(*fmerit_w_sm, 29491), fmerit_w, 3277));  /*Q14 */
    fmerit_w = *fmerit_w_sm;
    move16();

    tmp1 = fmerit;
    move16();
    if(sub(fmerit, 8192) < 0)
    {
        tmp1 = 16384;
        move16();
    }
    fmerit_m = negate(add(-32768, tmp1));
    *fmerit_m_sm = add(shr(*fmerit_m_sm, 1), shr(fmerit_m, 1));  /*Q14 */
    fmerit_m = *fmerit_m_sm;
    move16();

    pt1 = til;
    FOR( i = 0; i < NB_SUBFR; i++ )
    {
        tmp = sub(pitch_var_cur, 320);
        tmp = s_and(tmp, *pt1);
        if (tmp < 0 )
        {
            *pt1 = 1638;
            move16();
        }

        tmp1 = sub(8192, *pt1);
        *pt1 = s_max(6554, tmp1);
        move16();
        tmp1 = add(*ne_min, 7680); /*Q8  */
        tmp1 = mult_r(tmp1, 7340); /*Q20 - > 0.007  //Q13 */
        *pt1 = add(*pt1, tmp1);
        move16();/*Q13 */

        L_tmp = L_mult0(*pt1, fmerit_m); /*Q13+14 */
        *pt1++ = extract_l(L_shr(L_tmp, 14)); /*Q13 */
    }
    /* predict LPC coefficents and calculate sub-frame gains */
    p_Aq = Aq;
    p_Ap = Ap;
    pt1 = sub_gain;
    pt2 = til0;
    pt3 = til;
    pt6 = voice_factors;
    FOR( i = 0; i < NB_SUBFR; i++ )
    {
        envelope_fx( core_brate, p_Aq, p_Ap, pt1, *pt2, *pt3, *pt6, prev_r,
                     voice_facor_sm, unvoicing, unvoicing_sm, unvoicing_flag );
        pt1++;
        pt2++;
        pt3++;
        pt6++;

        p_Aq += (M+1);
        p_Ap += (M+1);
    }

    AdaptiveStartBand_fx( &start_band, core_brate, lsf_new, voice_fac, *amr_io_class, voicing_flag, start_band_old, OptCrit_old );


    q_tmp = Exp16Array(L_FRAME, exc);
    qdct = sub(q_tmp, 1);
    Copy_Scale_sig_16_32(exc, exc32, L_FRAME, qdct);

    qdct = add(qdct, Q_exc);
    edct_fx(exc32, dct_exc32, L_FRAME, &qdct);
    q_tmp = Exp32Array(L_FRAME, dct_exc32);

    q_tmp = sub(q_tmp,16);
    Copy_Scale_sig_32_16(dct_exc32, dct_exc, L_FRAME, q_tmp);
    qdct = add(qdct, q_tmp);

    set16_fx( dct_hb, 0, L_FRAME16k);
    pt1 = &dct_hb[200];
    pt2 = &dct_exc[200];
    FOR ( i = 200; i < 240; i++)
    {
        *pt1++ = *pt2++;
        move16();/*qdct */
    }
    set16_fx(signum, 1, L_SUBFR16k);
    pt1 = dct_hb+240;
    pt2 = dct_exc+start_band;
    pt3 = signum;
    FOR ( i = 240; i < L_FRAME16k; i++ )
    {
        if (*pt2<0)
        {
            *pt3 = -1;
            move16();
        }
        *pt1++ = abs_s(*pt2++);
        move16(); /*qdct */
        pt3++;
    }
    hb_ener = dot_prod_satcontr(&dct_hb[240], &dct_hb[240], qdct, qdct, &q1, L_SUBFR16k);

    L_tmp = L_shl(L_mult(start_band, 205), 14); /*Q30 */
    tmp = round_fx(L_tmp); /*Q14 */
    tmp = sub(18022, tmp); /*Q14 */
    fmerit_w = round_fx(L_shl(L_mult(fmerit_w, tmp), 1)); /*Q: 14+14+1+1-16 = 14 */


    L_tmp = L_deposit_l(fmerit_w); /*Q14 */
    L_tmp = Isqrt(L_tmp);  /*Q(31-7) */
    tmp = round_fx(L_tmp); /*Q8 */
    q2 = norm_s(tmp);
    alpha = div_s(shl(1, sub(14, q2)), tmp); /*Q(29-q2-8); */
    alpha = shl(alpha, sub(q2, 7));/*Q14 */

    beta = sub(16384, fmerit_w); /*Q14 */

    L_tmp = L_mult(alpha, 31130); /*Q30 */
    gamma = round_fx(L_tmp); /*Q14 */
    gamma = sub(17203, gamma); /*Q14 */
    gamma = s_min(16384, gamma);
    gamma = s_max(4915, gamma);

    IF ( sub(beta, 16384) < 0)
    {
        L_tmp = 1; /*variable for tonal energy*/

        pt1 = hb_amb;
        pt2 = hb_tonal;
        pt3 = &dct_hb[240];
        FOR (i=0; i<8; i++)
        {
            fn = add(i,8);
            tmp1 = div_s(1, fn); /*Q15  */
            tmp = 0;
            move16();
            pt4 = &dct_hb[240];
            FOR (j=0; j<fn; j++)
            {
                tmp = add(tmp, shr(*pt4++,2)); /*qdct-2 */
            }
            *pt1 = round_fx(L_shl(L_mult(tmp, tmp1),2)); /*qdct */
            *pt2 = sub(*pt3, *pt1);
            move16();
            IF ( *pt2 > 0 )
            {
                L_tmp = L_mac0(L_tmp, shr(*pt2,1), shr(*pt2,1));
            }
            pt1++;
            pt2++;
            pt3++;
        }
        FOR (; i < L_SUBFR16k-8; i++)
        {
            fb = sub(i,7);
            fn = add(fb,15);
            tmp = 0;
            pt4 = &dct_hb[fb+240];
            FOR (j=fb; j<fn; j++)
            {
                tmp = add(tmp, shr(*pt4++,2)); /*qdct-2 */
            }
            *pt1 = mult_r(tmp, 8738);
            move16(); /* 8738 = 1/15 in Q17 qdct */
            *pt2 = sub(*pt3, *pt1);
            move16();
            IF ( *pt2 > 0 )
            {
                L_tmp = L_mac0(L_tmp, shr(*pt2,1), shr(*pt2,1));
            }
            pt1++;
            pt2++;
            pt3++;
        }
        FOR (; i<L_SUBFR16k; i++)
        {
            fb = sub(i,7);
            tmp = 0;
            move16();
            fn = sub(L_SUBFR16k, fb);
            tmp1 = div_s(1, fn); /*Q15 */
            pt4 = &dct_hb[fb+240];
            FOR (j=fb; j<L_SUBFR16k; j++)
            {
                tmp = add(tmp, shr(*pt4++,2)); /*qdct-2 */
            }
            *pt1 = round_fx(L_shl(L_mult(tmp, tmp1),2)); /*qdct */
            *pt2 = sub(*pt3, *pt1);
            move16();
            IF ( *pt2 > 0 )
            {
                L_tmp = L_mac0(L_tmp, shr(*pt2,1), shr(*pt2,1)); /*2*qdct-2 */
            }
            pt1++;
            pt2++;
            pt3++;
        }

        /*scaling of hb_ener is q1; tonal_ener<hb_ener, so scaling of tonal_ener can be q1 too*/
        tonal_ener = round_fx(L_shr(L_tmp, sub(shl(qdct,1), add(q1,18))));
        tmp = sub(hb_ener, tonal_ener); /*q1 */
        tmp1 = round_fx(L_shl(L_mult(beta, tonal_ener),1)); /*q1 */
        tmp1 = sub(hb_ener, tmp1); /*q1 */
        tmp = div_s(tmp, s_max(tmp1,1)); /*Q15 */
        g = shl(mult(beta, tmp),1); /*Q15 */
        q2 = norm_s(g);
        inv_g = div_s(shl(1, sub(14, q2)), g); /*Q(29-q2-15); */
        inv_g = shl(inv_g, sub(q2, 2));/*Q12 */
        pt1 = hb_tonal;
        pt2 = hb_amb;
        pt3 = &dct_hb[240];
        pt4 = signum;
        FOR (i=0; i<L_SUBFR16k; i++)
        {
            if (*pt1>0)
            {
                *pt1 = mult_r(*pt1,g); /*qdct  */ move16();
            }
            *pt2 = round_fx(L_shl(L_mult(*pt2,inv_g),3)); /*qdct */
            *pt3 = add(*pt1, *pt2);
            move16();
            *pt3 = extract_l(L_mult0(*pt3, *pt4)); /*qdct   */
            pt1++;
            pt2++;
            pt3++;
            pt4++;
        }

        ener = dot_prod_satcontr(&dct_hb[240], &dct_hb[240], qdct, qdct, &q2, L_SUBFR16k);
        scale = div_s(shl(1, 14), hb_ener); /*Q(29-q1) */
        L_tmp = L_mult(ener, scale); /*30-q1+q2 */
        q2 = sub(q1, q2); /*30-q2 */
        scale = round_fx(Isqrt(L_shl(L_tmp, sub(q2, 24))));  /*Q12  */
        scale = round_fx(L_shl(L_mult(scale, gamma),4)); /*Q15 */
    }
    ELSE
    {
        scale = 32767; /*~1 in Q15 */ move16();
    }

    IF ( L_sub(core_brate, ACELP_6k60) == 0 )
    {
        filt_weight_coeff = 60;
        move16();
        rev_filt_weight_coeff = 555;
        move16(); /* 1/(filt_weight_coeff-1) Q15 */
    }
    ELSE IF ( L_sub(core_brate, ACELP_8k85) == 0 )
    {
        filt_weight_coeff = 40;
        move16();
        rev_filt_weight_coeff = 840;
        move16();
    }
    ELSE
    {
        filt_weight_coeff = 20;
        move16();
        rev_filt_weight_coeff = 1725;
        move16();
    }

    pt1 = filt_weight;
    FOR ( i = 0; i < filt_weight_coeff; i++)
    {
        L_tmp = L_mult(-32735, rev_filt_weight_coeff); /*Q31 */
        L_tmp = L_shl(Mult_32_16(L_tmp, i), 14); /*Q16+14 */
        *pt1++ = add(round_fx(L_tmp), 16384);
        move16();/*Q14 */
    }

    IF ( L_sub(core_brate, ACELP_23k85) == 0 )
    {
        pt1 = dct_hb+240;
        tmp = sub(filt_weight_coeff, 80);
        pt3 = filt_weight+tmp;
        FOR( i = 240; i < L_FRAME16k; i++ )
        {
            *pt1 = mult_r(*pt1, scale); /*qdct */ move16();

            IF ( sub(i, sub(320, filt_weight_coeff)) >= 0 )
            {
                *pt1 = round_fx(L_shl(L_mult(*pt3, *pt1), 1)); /*qdct */
            }
            pt1++;
            pt3++;
        }
        pt1 = dct_hb+200;
        pt6 = filt_hp_fx;
        FOR( i = 200; i < 256; i++ )
        {
            *pt1 = mult_r(*pt6++, *pt1);
            move16(); /*qdct */
            pt1++;
        }
        pt1 = HF_corr_gain;
        pt6 = hf_gain;
        FOR ( i = 0; i < NB_SUBFR; i++)
        {
            tmp = *pt6++;
            move16();
            *pt1++ = HP_gain_fx[tmp];
            move16();
        }
    }
    ELSE
    {
        pt1 = dct_hb+240;
        tmp = sub(filt_weight_coeff, 80);
        pt3 = filt_weight+tmp;
        FOR( i = 240; i < L_FRAME16k; i++ )
        {
            *pt1 = mult_r(*pt1, scale); /*qdct */
            IF ( sub(i, 255) > 0 )
            {
                *pt1 = mult_r(19505, *pt1);
                move16();
            }

            IF ( sub(i, sub(320, filt_weight_coeff)) >= 0 )
            {
                *pt1 = round_fx(L_shl(L_mult(*pt3, *pt1), 1)); /*qdct */
            }
            pt1++;
            pt3++;
        }

        pt1 = dct_hb+200;
        pt6 = filt_hp_fx;
        pt7 = deem_tab_fx;
        FOR( i = 200; i < 256; i++ )
        {
            *pt1 = mult_r(*pt6++, *pt1);
            move16();/*qdct */
            *pt1 = mult_r(*pt7++, *pt1);
            move16();
            pt1++;
        }
    }

    q_tmp = Exp16Array(L_FRAME16k, dct_hb);
    qhf = sub(q_tmp, 1);
    Copy_Scale_sig_16_32(dct_hb, dct_hb32, L_FRAME16k, qhf);
    qhf = add(qhf, qdct);
    edct_fx(dct_hb32, exc16k32, L_FRAME16k, &qhf);
    q_tmp = Exp32Array(L_FRAME16k, exc16k32);
    q_tmp = sub(q_tmp,16);
    Copy_Scale_sig_32_16(exc16k32, exc16k, L_FRAME16k, q_tmp);
    qhf = add(qhf, q_tmp);

    ener = dot_prod_satcontr(exc, exc, Q_exc, Q_exc, &q1, L_FRAME);
    tmp = dot_prod_satcontr(exc16k, exc16k, qhf, qhf, &q2, L_FRAME16k);

    pt6 = exc;
    pt2 = exc16k;

    FOR ( i = 0; i < NB_SUBFR; i++ )
    {
        e_subfr1 = dot_prod_satcontr(pt6, pt6, Q_exc, Q_exc, &q3, L_SUBFR);
        e_subfr2 = dot_prod_satcontr(pt2, pt2, qhf, qhf, &q4, L_SUBFR16k);

        L_tmp = L_mult(e_subfr1, tmp); /*Q(q2+q3+1) */
        q3 = add(add(q2, q3), 1);
        shift = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, shift); /*Q(q3+shift); */
        q3 = add(q3, shift);
        scale = round_fx(L_tmp); /*Q(q3-16); */
        q3 = sub(q3, 16);
        scale = div_s(shl(1, 14), scale); /*Q(29-q3)  */
        L_tmp = L_mult(scale, ener); /*Q(29-q3+q1+1) */
        shift = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, shift); /*Q(29-q3+q1+1+shift) */
        scale = round_fx(L_tmp); /*Q(29-q3+q1+1+shift-16) */
        L_tmp = L_mult(scale, e_subfr2); /*Q(29-q3+q1+1+shift-16+q4+1)=Q(15+q1-q3+q4+shift) */
        q3 = sub(15, q3);
        q3 = add(q3, q1);
        q3 = add(q3, shift);
        q3 = add(q3, q4);
        scale = round_fx(Isqrt(L_shl(L_tmp, sub(6, q3)))); /*Q12 */

        pt6 += L_SUBFR;
        FOR ( j = 0; j < L_SUBFR16k; j++ )
        {
            *pt2 = round_fx(L_shl(L_mult(*pt2, scale), 2)); /*qhf-1 */
            pt2++;
        }
    }
    qhf = sub(qhf, 1);

    p_Ap = Ap;
    pt1 = exc16k;
    pt2 = synth_out;
    pt3 = sub_gain;
    pt4 = HF_corr_gain;
    pt5 = til0;
    pt6 = voice_factors;
    pt7 = exc;

    FOR( i = 0; i < NB_SUBFR; i++ )
    {
        hf_synthesis_amr_wb_fx( core_brate, output_subfr, p_Ap, pt1, pt2,
                                mem_syn_hf, delay_syn_hf, mem_hp_interp, *pt3, *pt4, *pt5, *pt6, pt7,
                                Q_exc, Q_out, qhf);
        p_Ap += (M+1);
        pt1 += L_SUBFR16k;
        pt2 += output_subfr;
        pt3++;
        pt4++;
        pt5++;
        pt6++;
        pt7 += L_SUBFR;
    }

    return;
}
static void hf_synthesis_amr_wb_fx(
    const Word32 core_brate,                 /* i  : core bitrate : Q0                   */
    const Word16 output_subfr,               /* i  : output sub-frame length : Q0        */
    const Word16 Ap[],                       /* i  : quantized Aq : Q12                  */
    Word16 exc16k[],                   /* i  : excitation at 16 kHz : Qhf          */
    Word16 synth_out[],                /* i/o: synthesis signal at output Fs : Qo  */
    Word16 *mem_syn_hf,                /* i/o: HF synthesis memory : Qo            */
    Word16 *delay_syn_hf,              /* i/o: HF synthesis memory : Qo            */
    Word16 *mem_hp_interp,             /* i/o: interpol. memory  : Qo              */
    Word16 p_r,                        /* i  : sub-frame gain : Q12                */
    Word16 HF_corr_gain,               /* i  : HF gain index : Q14 */
    Word16 til,                        /*Q14*/
    Word16 voice_factors,              /*Q14*/
    const Word16 exc[],                      /* i  : excitation at 12.8 kHz : Qi          */
    const Word16 Q_exc,                         /*exc scaling*/
    const Word16 Q_out,                         /*synth_out scaling*/
    Word16 qhf                         /*exc16k scaling*/
)
{
    Word16 i, delay;
    Word16 HF_syn[L_SUBFR16k], upsampled_HF_syn[L_SUBFR48k];
    Word16 temp_buffer[NS2SA(16000,DELAY_CLDFB_NS)];
    Word16 ener, tmp, scale, exc2385[L_SUBFR16k];
    Word32 L_tmp;
    Word16 q1, q2,q3, shift;
    Word16 *pt1, *pt2, flag;
    IF ( L_sub(core_brate, ACELP_23k85) == 0 )
    {
        ener = dot_prod_satcontr(exc, exc, Q_exc, Q_exc, &q1, L_SUBFR);
        tmp = dot_prod_satcontr(exc16k, exc16k, qhf, qhf, &q2, L_SUBFR16k);

        L_tmp = L_mult(ener, 6554); /*Q(q1+16) */
        q3 = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, q3); /*Q(q1+q3+16) */
        ener = extract_h(L_tmp); /*Q(q1+q3); */
        q1 = add(q1, q3);

        scale = div_s(shl(1, 14), ener); /*Q(29-q1) */
        L_tmp = L_mult(tmp, scale); /*30-q1+q2 */
        q2 = sub(q1, q2); /*30-q2 */
        scale = round_fx(Isqrt(L_shl(L_tmp, sub(q2, 24))));  /*Q12  */

        pt1 = exc16k;
        pt2 = exc2385;
        FOR ( i = 0; i < L_SUBFR16k; i++ )
        {
            L_tmp = L_mult(*pt1++, HF_corr_gain); /*qhf+15*/
            L_tmp = Mult_32_16(L_tmp, scale); /*qhf-1+12+1*/
            *pt2++ = round_fx(L_shl(L_tmp, 1)); /*qhf-3*/
        }

        pt1 = exc16k;
        FOR ( i = 0; i < L_SUBFR16k; i++ )
        {
            *pt1 = mult_r(*pt1, p_r); /*qhf-3*/ move16();
            pt1++;
        }

        qhf = sub(qhf, 3);

        ener = dot_prod_satcontr(exc16k, exc16k, qhf, qhf, &q1, L_SUBFR16k);
        tmp = dot_prod_satcontr(exc2385, exc2385, qhf, qhf, &q2, L_SUBFR16k);
        L_tmp = L_mult(ener, 9830); /*Q(q1+16) */
        q3 = norm_l(L_tmp);
        L_tmp = L_shl(L_tmp, q3); /*Q(q1+q3+16) */
        ener = extract_h(L_tmp); /*Q(q1+q3); */
        q1 = add(q1, q3);

        scale = div_s(shl(1, 14), ener); /*Q(29-q1) */
        L_tmp = L_mult(tmp, scale); /*30-q1+q2 */
        q2 = sub(q1, q2); /*30-q2 */
        scale = round_fx(Isqrt(L_shl(L_tmp, sub(q2, 24))));  /*Q12  */

        flag = negate(s_and(til,-0x8000));
        if (sub(scale, 4096) > 0)
        {
            flag = 1;
            move16();
        }
        IF ( flag )
        {
            Copy( exc2385, exc16k, L_SUBFR16k );
        }
        ELSE
        {
            pt1 = exc16k;
            pt2 = exc2385;
            FOR ( i = 0; i < L_SUBFR16k; i++ )
            {
                tmp = sub(16348, shl(til, 1));/*Q14 */
                L_tmp = L_mult(tmp, sub(26214, shr(voice_factors, 1))); /*Q29*/
                tmp = round_fx(L_shr(L_tmp, 1)); /*Q12*/
                tmp = s_min(tmp, 4096);
                tmp = s_max(tmp, scale);
                *pt1++ = round_fx(L_shl(L_mult(*pt2++, tmp), 3))/*qhf*/;
            }
        }

    }
    ELSE
    {
        pt1 = exc16k;
        FOR ( i = 0; i < L_SUBFR16k; i++ )
        {
            *pt1 = mult_r(*pt1, p_r); /*qhf-3*/ move16();
            pt1++;
        }

        qhf = sub(qhf, 3);
    }

    shift = sub(qhf, Q_out);
    Syn_filt_s( shift, Ap, M, exc16k, HF_syn, L_SUBFR16k, mem_syn_hf + (M16k - M), 1 ); /*Q_out=qhf-shift */

    /*-----------------------------------------------------------------*
     * Resample to output sampling rate
     * Synchronize LB and HB components (delay componsation)
     * Add synthesised high band to speech synthesis
     *-----------------------------------------------------------------*/

    /* compensate CLDFB resampling delay */
    delay = NS2SA(16000,DELAY_CLDFB_NS);
    Copy( HF_syn+L_SUBFR16k-delay, temp_buffer, delay );
    Copy( HF_syn, HF_syn+delay, L_SUBFR16k-delay );
    Copy( delay_syn_hf, HF_syn, delay );
    Copy( temp_buffer, delay_syn_hf, delay );

    IF( sub(output_subfr, L_SUBFR48k) == 0 )   /* 48kHz sampled output */
    {
        Word16 s;
        s = s_max(s_min(sub(s_min(Find_Max_Norm16(HF_syn, L_SUBFR16k), Find_Max_Norm16(mem_hp_interp, INTERP_3_1_MEM_LEN - 3)), 3),
                        sub(Find_Max_Norm16(mem_hp_interp + INTERP_3_1_MEM_LEN - 3, 3), 1)), 0 );
        Scale_sig( HF_syn, L_SUBFR16k, s );
        Scale_sig( mem_hp_interp, INTERP_3_1_MEM_LEN, s );

        interpolate_3_over_1_allpass_fx( HF_syn, L_SUBFR16k, upsampled_HF_syn, mem_hp_interp, allpass_poles_3_ov_2 );

        Scale_sig( upsampled_HF_syn, L_SUBFR48k, add(-s,-1) );
        Scale_sig( mem_hp_interp, INTERP_3_1_MEM_LEN, -s );
        Scale_sig( HF_syn, L_SUBFR16k, -s );
    }
    ELSE IF( sub(output_subfr, L_SUBFR32k) == 0 )  /* 32kHz sampled output */
    {
        Word16 s;
        s = s_max( sub(s_min(Find_Max_Norm16(HF_syn, L_SUBFR16k), Find_Max_Norm16(mem_hp_interp, 2*ALLPASSSECTIONS_STEEP)),2), 0 );
        Scale_sig( HF_syn, L_SUBFR16k, s );
        Scale_sig( mem_hp_interp, 2*ALLPASSSECTIONS_STEEP, s );

        Interpolate_allpass_steep_fx( HF_syn, mem_hp_interp, L_SUBFR16k, upsampled_HF_syn );

        Scale_sig( upsampled_HF_syn, 2*L_SUBFR16k, -s );
        Scale_sig( mem_hp_interp, 2*ALLPASSSECTIONS_STEEP, -s );
        Scale_sig( HF_syn, L_SUBFR16k, -s );
    }
    ELSE    /* 16kHz sampled output */
    {
        Copy( HF_syn, upsampled_HF_syn, L_SUBFR16k );
    }

    Vr_add( synth_out, upsampled_HF_syn, synth_out, output_subfr);

    return;
}

static Word16 EnhanceClass_fx(
    const Word16 qq_fx,
    const Word16 pp_fx,
    const Word16 tilt0_fx,           /* i  : spectrum tilt                */
    const Word16 tilt_fx,            /* i  : spectrum tilt                */
    const Word16 voice_factor_fx,    /* i  : voice factor                 */
    Word16 *voice_fac_fx,      /* i/o: smoothed voiced parameter    */
    Word16 *unvoicing_fx,      /* i/o: unvoiced parameter           */
    Word16 *unvoicing_sm_fx,   /* i/o: smoothed unvoiced parameter  */
    Word16 *unvoicing_flag  /* i/o: unvoiced flag                */
)
{
    Word16 unvoicing_tmp_fx;
    Word16 tmp, tmp1;
    Word32 L_tmp;

    /* Decide (*unvoicing_flag) to allow BWE enhancement when qq>pp */
    /**voice_fac_fx = add(mult_r(*voice_fac_fx, 24576), mult_r(voice_factor_fx, 8192));  //Q15 */
    *voice_fac_fx = round_fx(L_mac(L_mult(*voice_fac_fx, 24576), voice_factor_fx, 8192));  /*Q15 */

    tmp = mult_r(sub(8192, tilt0_fx), 16384); /*Q13 */

    L_tmp = L_sub(32768, *voice_fac_fx); /*Q15 */

    L_tmp = Mult_32_16(L_tmp, tmp); /*Q13 */
    tmp = extract_l(L_tmp); /*Q13 */

    tmp1 = mult_r(tilt_fx, 21845); /*Q15->1/1.5 ->Q13+15-15->Q13 */
    tmp1 = s_min(tmp1, 8192);

    L_tmp = L_mult(tmp, tmp1); /*Q13+Q13+1 */
    unvoicing_tmp_fx = extract_l(L_shr(L_tmp, 12)); /*Q15 */

    /**unvoicing_fx = add(mult_r(16384, *unvoicing_fx), mult_r(16384, unvoicing_tmp_fx));  //Q15 */
    *unvoicing_fx = round_fx(L_mac(L_mult(16384, *unvoicing_fx), 16384, unvoicing_tmp_fx));  /*Q15 */

    IF( sub(*unvoicing_sm_fx, *unvoicing_fx) > 0 )
    {
        /**unvoicing_sm_fx = add(mult_r(29491, *unvoicing_sm_fx), mult_r(3277, *unvoicing_fx));  //Q15 */
        *unvoicing_sm_fx = round_fx(L_mac(L_mult(29491, *unvoicing_sm_fx), 3277, *unvoicing_fx));  /*Q15 */
    }
    ELSE
    {
        /**unvoicing_sm_fx = add(mult_r(32440, *unvoicing_sm_fx), mult_r(328, *unvoicing_fx));  //Q15 */
        *unvoicing_sm_fx = round_fx(L_mac(L_mult(32440, *unvoicing_sm_fx), 328, *unvoicing_fx));  /*Q15 */
    }

    if ( sub(sub(*unvoicing_fx, *unvoicing_sm_fx),3277) > 0)
    {
        *unvoicing_flag = 1;
    }

    if ( sub(sub(*unvoicing_fx, *unvoicing_sm_fx),1638) < 0)
    {
        *unvoicing_flag = 0;
    }
    test();
    return ( *unvoicing_flag && sub(qq_fx, pp_fx)>0 );
}

static void envelope_fx(
    const Word32 core_brate,      /* i  : core bitrate                    */
    const Word16 Aq_dyn_scal[],   /* i  : de-quant. LPC coefficents, dynamic scaling  */
    Word16 Ap[],            /* o  : extended LPC coefficents, Q12   */
    Word16 *sub_gain,       /* o  : sub-frame gain, Q12             */
    Word16 tilt0,           /* i  : spectrum tilt, Q14              */
    Word16 tilt,            /* i  : spectrum tilt, Q13              */
    Word16 voice_factor,    /* i  : voice factor, Q15               */
    Word16 *prev_r,         /* i/o: previous sub-frame gain, Q10    */
    Word16 *voice_fac,      /* i/o: smoothed voiced parameter, Q15  */
    Word16 *unvoicing,      /* i/o: unvoiced parameter, Q15         */
    Word16 *unvoicing_sm,   /* i/o: smoothed unvoiced parameter, Q15*/
    Word16 *unvoicing_flag  /* i/o: unvoiced flag                   */
)
{

    Word16 px, py, rx, ry, pp, rr, qx, qy, qq; /*Q10*/
    Word16 i, Unvoicing_flag;
    Word16 alpha; /*Q14*/
    Word16 est_level1, est_level2; /*Q10*/
    Word32 L_tmp;
    Word16 tmp, q1, q2, q3, shift;
    Word16 As[3], k1, k2;
    Word16 *pt1;
    const Word16 *pt2, *pt3;
    Word16 Aq[M+1];


    /* Aq has dynamic scaling
      go back to Q12 to make sure there's no overflow while calculating qx,qy*/
    shift = sub(norm_s(Aq_dyn_scal[0]),2);
    Copy_Scale_sig(Aq_dyn_scal, Aq, M+1, shift);

    /* LPC envelope weighting */
    IF( L_sub(core_brate,ACELP_6k60) == 0 )
    {
        weight_a_lc_fx( Aq, Ap, Gamma_29491_Tbl, M );
    }
    ELSE
    {
        weight_a_lc_fx( Aq, Ap, Gamma_19661_Tbl_fx, M );
    }
    /* Ap has dynamic scaling
      go back to Q12 to make sure there's no overflow while calculating px,py*/
    shift = sub(norm_s(Ap[0]),2);
    IF(shift != 0)
    {
        Scale_sig(Ap, M+1, shift);
    }

    /* LPC envelope level estimate */
    L_tmp = L_deposit_l(0);
    pt1 = Ap;
    pt2 = exp_tab_p_fx;
    FOR ( i = 0; i <= M; i++ )
    {
        L_tmp = L_mac(L_tmp, *pt1++, *pt2++);
    }
    q1 = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, q1);/*Q(27+q1)*/
    px = round_fx(L_shr(L_tmp, 1)); /*Q(10+q1)*/

    L_tmp = L_deposit_l(0);

    pt1 = Ap;
    pt2 = exp_tab_p_fx+33;
    FOR ( i = 0; i <= M; i++ )
    {
        L_tmp = L_mac(L_tmp, *pt1++, *pt2--);
    }
    q2 = norm_l(L_tmp);
    shift = sub(q1, q2);
    IF ( shift >= 0 )
    {
        px = shr(px, shift);
        L_tmp = L_shl(L_tmp, q2);/*Q(27+q2)*/
        q1 = q2;
        move16();
    }
    ELSE
    {
        L_tmp = L_shl(L_tmp, q1);/*Q(27+q1)*/
    }
    py = round_fx(L_shr(L_tmp, 1)); /*Q(10+q1)*/

    L_tmp = L_deposit_l(0);
    pt2 = Aq;
    pt3 = exp_tab_q_fx;
    FOR ( i = 0; i <= M; i++ )
    {
        L_tmp = L_mac(L_tmp, *pt2++, *pt3++);
    }
    q2 = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, q2);/*Q(27+q2)*/
    rx = round_fx(L_shr(L_tmp, 1)); /*Q(10+q2)*/

    L_tmp = L_deposit_l(0);
    pt2 = Aq;
    pt3 = exp_tab_q_fx+33;
    FOR ( i = 0; i <= M; i++ )
    {
        L_tmp = L_mac(L_tmp, *pt2++, *pt3--);
    }
    q3 = norm_l(L_tmp);
    shift = sub(q2, q3);
    IF ( shift >= 0 )
    {
        rx = shr(rx, shift);
        L_tmp = L_shl(L_tmp, q3);/*Q(27+q3)*/
        q2 = q3;
        move16();
    }
    ELSE
    {
        L_tmp = L_shl(L_tmp, q2);/*Q(27+q2)*/
    }
    ry = round_fx(L_shr(L_tmp, 1)); /*Q(10+q2)*/

    L_tmp = L_mult(px, px);
    L_tmp = L_mac(L_tmp, py, py); /*Q(21+2*q1)*/
    pp = round_fx(Isqrt(L_shr(L_tmp, add(11, shl(q1, 1))))); /*Q10*/

    L_tmp = L_mult(rx, rx);
    L_tmp = L_mac(L_tmp, ry, ry); /*Q(21+2*q1)*/
    rr = round_fx(Isqrt(L_shr(L_tmp, add(11, shl(q2, 1))))); /*Q10*/

    Copy(Aq, As, 3);
    IF ( add(2048, shr(As[2],1)) == 0 )
    {
        k2 = -2458;
        move16();
        k1 = 4055;
        move16();
        if (As[1]<0)
        {
            k1 = -k1;
            move16();
        }
    }
    ELSE
    {
        k1 = add(2048, shr(As[2],1)); /*Q11 */
        q1 = 11;
        move16();
        q2 = norm_s(k1);
        k1 = (shl(k1, q2)); /*q1+q2 */
        tmp = abs_s(k1);
        q1 = add(q1, q2);
        tmp = div_s(shl(1, 14), tmp); /*Q(29-q1) */
        if ( k1 < 0 )
        {
            tmp = negate(tmp);
            move16();
        }

        L_tmp = L_mult(As[1], tmp); /*Q(42-q1) */
        q1 = sub(q1, 14);
        k1 = round_fx(L_shl(L_tmp, q1)); /*Q12 */
        k2 = As[2];
        move16(); /*Q12 */
        if ( sub(k2, 2458) > 0 )
        {
            k2 = 2458;
            move16();
        }
        if ( add(k2, 2458) < 0 )
        {
            k2 = -2458;
            move16();
        }
        if ( sub(k1, 4055) > 0 )
        {
            k1 = 4055;
            move16();
        }
        if ( add(k1, 4055) < 0 )
        {
            k1 = -4055;
            move16();
        }
    }
    As[1] = add(4096, k2);
    move16();
    L_tmp = L_mult(As[1], k1); /*Q25 */
    As[1] = round_fx(L_shl(L_tmp, 3)); /*Q12 */
    As[2] = k2;
    move16(); /*Q12 */

    L_tmp = L_deposit_l(0);
    pt1 = As;
    pt2 = exp_tab_q_fx;
    FOR ( i = 0; i < 3; i++ )
    {
        L_tmp = L_mac(L_tmp, *pt1++, *pt2++);
    }
    q1 = norm_l(L_tmp);
    L_tmp = L_shl(L_tmp, q1);/*Q(27+q1)*/
    qx = round_fx(L_shr(L_tmp, 1)); /*Q(10+q1)*/


    L_tmp = L_deposit_l(0);
    pt1 = As;
    pt2 = exp_tab_q_fx+33;
    FOR ( i = 0; i < 3; i++ )
    {
        L_tmp = L_mac(L_tmp, *pt1++, *pt2--);
    }
    q2 = norm_l(L_tmp);
    shift = sub(q1, q2);
    IF ( shift >= 0 )
    {
        qx = shr(qx, shift);
        L_tmp = L_shl(L_tmp, q2);/*Q(27+q2)*/
        q1 = q2;
        move16();
    }
    ELSE
    {
        L_tmp = L_shl(L_tmp, q1);/*Q(27+q1)*/
    }
    qy = round_fx(L_shr(L_tmp, 1)); /*Q(10+q1)*/


    L_tmp = L_mult(qx, qx);
    L_tmp = L_mac(L_tmp, qy, qy);
    qq = round_fx(Isqrt(L_shr(L_tmp, add(11, shl(q1, 1))))); /*Q10*/

    Unvoicing_flag = EnhanceClass_fx( rr, pp, tilt0, tilt, voice_factor, voice_fac, unvoicing, unvoicing_sm, unvoicing_flag );

    alpha  = 0;
    move16();
    IF ( Unvoicing_flag )
    {
        IF ( sub(rr, (*prev_r)) > 0 )
        {
            rr = shr(add(rr, (*prev_r)), 1);
        }

        *prev_r = rr;
        move16();

        L_tmp = L_mult(tilt, sub(26214, shr(voice_factor, 1))); /*Q28*/

        L_tmp = L_min(L_tmp, 268435456);

        L_tmp = Mult_32_16(L_tmp, rr); /*Q23*/
        rr = round_fx(L_shl(L_tmp, 3)); /*Q10*/
        L_tmp = L_mult(tilt, sub(26214, shr(voice_factor, 1))); /*Q28*/
        L_tmp = L_max(L_tmp, 268435456);
        L_tmp = Mult_32_16(L_tmp, qq); /*Q23*/
        qq = round_fx(L_shl(L_tmp, 3)); /*Q10*/
        rr = s_min(rr, qq);
        rr= s_max(rr, pp);
    }
    ELSE
    {
        test();
        IF ( sub(rr, 1024) < 0 && sub((*prev_r), 1024) < 0 )
        {
            L_tmp = L_mult(rr, rr); /*Q21*/
            tmp = round_fx(L_shl(L_tmp, 9)); /*Q14*/
            L_tmp = L_sub(2097152, L_tmp);  /*Q21*/
            alpha = round_fx(L_shl(L_tmp, 9)); /*Q14*/
            L_tmp = L_mult(alpha, (*prev_r)); /*Q25*/
            L_tmp = L_mac(L_tmp, tmp, rr);     /*Q25*/
            rr = round_fx(L_shl(L_tmp, 1));   /*Q10*/
        }

        *prev_r = rr;
        move16();

        L_tmp = L_mult(tilt, sub(26214, shr(voice_factor, 1))); /*Q28*/

        L_tmp = L_min(L_tmp, 268435456);
        L_tmp = Mult_32_16(L_tmp, qq); /*Q23*/
        est_level1 = round_fx(L_shl(L_tmp, 3)); /*Q10*/

        tmp = pp;
        move16();
        tmp = s_min(tmp, qq);
        rr = s_min(tmp, rr);

        L_tmp = L_mult(abs_s(sub(tilt, 8192)), sub(26214, shr(voice_factor, 1))); /*Q28*/
        L_tmp = L_add(L_tmp, 268435456);
        L_tmp = Mult_32_16(L_tmp, rr); /*Q23*/
        est_level2 = round_fx(L_shl(L_tmp, 3)); /*Q10*/

        rr = s_min(est_level1, est_level2);
    }


    q1 = norm_s(pp);
    tmp = div_s(shl(1, sub(14, q1)), pp); /*Q(29-q1-10) */
    L_tmp = L_mult(rr, tmp);/*Q(30-q1-10+10) */

    *sub_gain = s_min(20480,round_fx(L_shl(L_tmp, sub(q1, 2)))); /*Q12 */

    return;

}

/*---------------------------------------------------------------------*
 * AdaptiveStartBand_fx()
 *
 * adaptively select the start band of bandwidth extension
 *---------------------------------------------------------------------*/

void AdaptiveStartBand_fx(
    Word16 *start_band,       /* o  : start point of copied band                */
    const Word32  rate,             /* i  : core bitrate                              */
    const Word16 *lsf_fx,           /* i  : Q2  lsf frequency                             */
    const Word16 voicing_fac_fx,    /* i  : Q14 voicing factors                           */
    const Word16 clas, 	            /* i  : signal class (determined by FEC algorithm)*/
    Word16 *voicing_flag,
    Word16 *start_band_old,
    Word32 *OptCrit_old_fx    /*i/o : Q15 */
)
{
    Word16 i, pos, M2, voicing_flag_old;
    Word16 lsf_diff_fx[M];
    Word16 W_fx;
    Word16 tmp1, tmp2;
    Word32 L_tmp;
    Word32 OptCrit_fx = 32768, Crit_fx;
    Word16 *pt1, flag;
    const Word16 *pt2, *pt3;

    /*voicing switching flag : to avoid switching start band frequently in VOICED or AUDIO area*/
    voicing_flag_old = *voicing_flag;
    test();
    test();
    test();
    if( sub(voicing_fac_fx, 6554) > 0 || (sub(voicing_fac_fx, 4915) > 0 && sub(clas,VOICED_CLAS) >= 0) || sub(clas,AUDIO_CLAS) == 0 )
    {
        *voicing_flag = 1;
        move16();
    }

    test();
    if( sub(voicing_fac_fx, 3277) < 0 && sub(clas,VOICED_CLAS) < 0 )
    {
        *voicing_flag = 0;
        move16();
    }

    /* rate adaptive start band */
    *start_band = 160;
    move16();
    IF( L_sub(rate, ACELP_23k05) < 0 )
    {
        pt1 = lsf_diff_fx+1;
        pt2 = lsf_fx+1;
        pt3 = lsf_fx;
        FOR(i=1; i<(M-1); i++)
        {
            *pt1++ = sub(*pt2++, *pt3++);
            move16();/*Q2 */
        }
        tmp1 = extract_l(Mult_32_16(rate, 27046)); /*Q14 */
        L_tmp = L_shr(L_mult0(tmp1, 22370), 15);  /*Q27->1/6000 ->Q26 */
        tmp2 = extract_l(L_tmp); /*Q26 */
        W_fx = mult_r(tmp1, tmp2); /*Q25 */

        if (sub(clas,AUDIO_CLAS) == 0)
        {
            W_fx = mult_r(W_fx, 24576);  /*Q25 */
        }

        pos = 2;
        move16();
        M2 = sub(M, 2);
        IF( sub(*voicing_flag,1) == 0 )
        {
            IF( L_sub(rate, ACELP_8k85) <= 0 )
            {
                M2 = sub(M, 8);
            }
            ELSE IF( L_sub(rate, ACELP_12k65) <= 0 )
            {
                M2 = sub(M, 6);
            }
            ELSE IF( L_sub(rate, ACELP_15k85) <= 0 )
            {
                M2 = sub(M, 4);
            }
        }

        /*do the procedure for i==2*/
        L_tmp = L_max(L_msu(171798692,  lsf_fx[2], W_fx), 171799);  /* Q2.56+25+1 */
        Crit_fx = Mult_32_16(L_tmp, lsf_diff_fx[2]); /* Q2.56+25+1+2.56-15 = Q11+2.56+2.56 */

        OptCrit_fx = L_add(Crit_fx, 0);
        pos = 2;
        move16();
        /*----------------------------------------------------------------*/

        pt2 = &lsf_fx[3];
        pt1 = &lsf_diff_fx[3];
        FOR(i=3; i<M2; i++)
        {
            L_tmp = L_max(L_msu(171798692,  *pt2++, W_fx), 171799);  /* Q2.56+25+1 */

            Crit_fx = Mult_32_16(L_tmp, *pt1++); /* Q2.56+25+1+2.56-15 = Q11+2.56+2.56 */

            IF( L_sub(Crit_fx, OptCrit_fx) <= 0 )
            {
                OptCrit_fx = L_add(Crit_fx, 0); /* Q11+2.56+2.56 */
                pos = i;
                move16();
            }
        }

        tmp1 = add(mult_r(lsf_fx[pos-1], 256), mult_r(lsf_fx[pos], 256));
        *start_band = s_min(s_max(sub(tmp1, 40), 40), 160);

        L_tmp = Mult_32_16(*OptCrit_old_fx, 22938); /* Q11+2.56+2.56 */
        test();
        test();
        test();
        test();
        IF ( sub(voicing_flag_old, *voicing_flag) != 0 || ( *voicing_flag == 0 && L_sub(OptCrit_fx, *OptCrit_old_fx) < 0 ) ||
             ( L_sub(OptCrit_fx, L_tmp) < 0 && L_sub(*OptCrit_old_fx, 858993) > 0 ) )
        {
            *OptCrit_old_fx = OptCrit_fx;
            move16();
            test();
            test();
            if ( sub(abs_s(sub((*start_band),(*start_band_old))), 20)<0 && sub(*voicing_flag,1)==0 && sub(voicing_flag_old,1)==0 )
            {
                *start_band = *start_band_old;
                move16();
            }
        }
        ELSE
        {
            test();
            if (L_sub(OptCrit_fx, (*OptCrit_old_fx))<0 && sub((*voicing_flag),1)==0)
            {
                *OptCrit_old_fx = OptCrit_fx;
                move16();
            }

            *start_band = *start_band_old;
            move16();
        }

        if (sub(clas,AUDIO_CLAS) == 0)
        {
            *start_band = s_min(*start_band, 120);
            move16();
        }

        flag = sub(s_and(*start_band, 0x0001),1);
        if (flag == 0)
        {
            *start_band = sub(*start_band, 1);
            move16();
        }
    }

    *start_band_old = *start_band;
    move16();

    return;
}

