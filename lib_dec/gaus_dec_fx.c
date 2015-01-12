/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "rom_com_fx.h"   /* Static table prototypes                */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"

/*---------------------------------------------------------------------*
   * Local functions
   *---------------------------------------------------------------------*/

void gaus_dec2v_fx( Decoder_State_fx *st_fx, Word16 *code,  const Word16 lg, const Word16 nb_bits);
static void dec_2pos_fx(Word16 index, Word16 *ind1, Word16 *ind2, Word16 *sign1, Word16 *sign2, Word16 log2_n);

/*---------------------------------------------------------------------*
 * gaus_dec()
 *
 * no adaptive excitation constructed
 * - decode the codebook indices,
 * - find the excitation
 *---------------------------------------------------------------------*/
void gaus_dec_fx(
    Decoder_State_fx *st_fx,         /* i/o: decoder static memory                                */
    const Word32 core_brate,         /* i   	: core bitrate                                    */
    const Word16 i_subfr,            /* i  		: subframe index                                  */
    Word16 *code,              /* o  		: unvoiced excitation                        Q12  */
    Word32 *L_norm_gain_code,  /* o  		: gain of normalized gaussian excitation     Q16  */
    Word16 *lp_gainp,          /* i/o		: lp filtered pitch gain(FER)                Q14  */
    Word16 *lp_gainc,          /* i/o		: lp filtered code gain (FER)                Q3   */
    Word16 *inv_gain_inov,     /* o  		: unscaled innovation gain                   Q12  */
    Word16 *tilt_code,         /* o  		: synthesis excitation spectrum tilt         Q15  */
    Word16 *voice_fac,         /* o  		: estimated voicing factor                   Q15  */
    Word16 *gain_pit,          /* o  		: pitch gain                                 Q14  */
    Word16 *pt_pitch_1,        /* o  		: floating pitch buffer						 Q6   */
    Word16 *exc,               /* o  		: excitation signal frame                         */
    Word32 *L_gain_code,       /* o  		: gain of the gaussian excitation            Q16  */
    Word16 *exc2,              /* o  		: Scaled excitation signal frame                  */
    Word16  *bwe_exc_fx,
    Word16 *sQ_exc,            /* i/o		: Excitation scaling factor (Decoder state)       */
    Word16 *sQsubfr            /* i/o		: Past excitation scaling factors (Decoder State) */
)
{
    Word16 i, exp, gain_code;
    Word16 idx, nb_bits;
    Word32 L_tmp;

    /*------------------------------------------------------------------------------------------*
     * Unvoiced : Gaussian codebook
     *------------------------------------------------------------------------------------------*/

    nb_bits = FCB_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, UNVOICED, i_subfr, TC_SUBFR2IDX_fx(-1))];
    move16();

    gaus_dec2v_fx( st_fx, code, L_SUBFR, shr(nb_bits,1));

    /*------------------------------------------------------------------------------------------*
     * - Gain of Gaussian excitation and normalized Gaussian excitation
     *------------------------------------------------------------------------------------------*/
    /* gain_inov = 1.0f / (float)sqrt((dot_product(code, code, L_SUBFR) + 0.01f) / L_SUBFR) */
    L_tmp = Dot_product12(code, code, L_SUBFR, &exp);
    exp = sub(exp, 24 + 6);          /* exp: -24 (code in Q12), -6 (/L_SUBFR) */
    L_tmp = Isqrt_lc(L_tmp, &exp);
    *inv_gain_inov = extract_h(L_shl(L_tmp, sub(exp, 3))); /* inv_gain_inov in Q12 */

    nb_bits = gain_bits_tbl[BIT_ALLOC_IDX_fx(core_brate, UNVOICED, i_subfr, TC_SUBFR2IDX_fx(-1))];
    move16();
    idx = (Word16)get_next_indice_fx( st_fx, nb_bits );
    move16();

    *L_gain_code = gain_dec_gaus_fx( idx, nb_bits,-30,190, *inv_gain_inov, L_norm_gain_code );

    /* update LP filtered gains for the case of frame erasures */
    lp_gain_updt_fx( i_subfr, 0, *L_norm_gain_code, lp_gainp, lp_gainc, L_FRAME );   /* supposes that gain_dec_gaus() is used for ACELP@12k8 only */

    /*------------------------------------------------------------------------------------------*
     * Updates
     *------------------------------------------------------------------------------------------*/

    *tilt_code = 0;
    move16();
    *voice_fac = -32768;  /* only unvoiced              */                             move16();
    *gain_pit  = 0;       /* needed for BASS postfitler */                             move16();
    *pt_pitch_1  = 4096;    /* floating pitch buffer  Q6    */                         move16();

    /*------------------------------------------------------------------------------------------*
     * Construct scaled excitation
     *------------------------------------------------------------------------------------------*/

    set16_fx(&exc[i_subfr],0, L_SUBFR);
    set16_fx(&exc2[i_subfr],0, L_SUBFR);

    IF( sub(st_fx->L_frame_fx,L_FRAME) == 0 )
    {
        Rescale_exc( st_fx->dct_post_old_exc_fx, &exc[i_subfr], &bwe_exc_fx[i_subfr * HIBND_ACB_L_FAC], st_fx->last_exc_dct_in_fx,
                     L_SUBFR, L_SUBFR* HIBND_ACB_L_FAC, *L_gain_code, sQ_exc, sQsubfr, exc2, i_subfr, UNVOICED );
    }
    ELSE
    {
        Rescale_exc( st_fx->dct_post_old_exc_fx, &exc[i_subfr], &bwe_exc_fx[i_subfr * 2], st_fx->last_exc_dct_in_fx,
        L_SUBFR, L_SUBFR* 2, *L_gain_code, sQ_exc, sQsubfr, exc2, i_subfr, UNVOICED );
    }

    gain_code = round_fx(L_shl(*L_gain_code, *sQ_exc));
    FOR (i = 0; i < L_SUBFR;  i++)
    {
        L_tmp = L_shl(L_mult(gain_code, code[i]), 3);
        exc[i+i_subfr] = round_fx(L_tmp);
    }

    return;
}



/*-----------------------------------------------------*
 * gaus_dec2v()
 *
 * decoder of Gaussian Codebook for unvoiced
 * consisting of addition of 2 Gaussian vectors
 *
 * One Gaussian vector of 190 values
 *-----------------------------------------------------*/

void gaus_dec2v_fx(
    Decoder_State_fx *st_fx,  /* i/o: decoder state structure           */
    Word16 *code,     /* o  : decoded gaussian vector   Q12-exp */
    const Word16 lg,        /* i  : codevector length         Q0      */
    const Word16 nb_bits    /* i  : nb ob bits per track (max 6)      */
)
{
    Word16 i, ind1, ind2,idx;
    Word16 step;
    Word16 sign1, sign2;
    Word16 delta, delta2, inv_delta;
    Word16 gaus_dico2_fx[190];
    Word16 tmp16;
    const Word16 *pt1, *pt2;
    Word16 index_delta;

    step = shr(0x80, nb_bits);

    idx = (Word16)get_next_indice_fx( st_fx, add(shl(nb_bits,1),1) );
    index_delta = (Word16)get_next_indice_fx( st_fx, 3 );

    dec_2pos_fx( idx, &ind1, &ind2, &sign1, &sign2, nb_bits );

    delta = shl(index_delta, STEP_DELTA_FX);
    delta2 = mac_r(16384*65536, shr(delta,1), delta);
    inv_delta = div_s(16384, delta2);

    IF (delta > 0)
    {
        gaus_dico2_fx[0] = gaus_dico_fx[0];
        move16(); /*Q12             */
        FOR (i=1; i<190; i++)
        {
            /* gaus_dico2[i] = (gaus_dico_fx[i] - delta*gaus_dico_fx[i-1])/(1+delta*delta) */
            tmp16  = msu_r(L_deposit_h(gaus_dico_fx[i]), delta, gaus_dico_fx[i-1]);
            gaus_dico2_fx[i] = mult_r(tmp16, inv_delta);
        }
    }
    ELSE
    {
        FOR (i=0; i<190; i++)
        {
            gaus_dico2_fx[i] = gaus_dico_fx[i];
            move16();  /*Q12 */
        }
    }

    pt1 = &gaus_dico2_fx[i_mult2(ind1, step)];
    move16();
    pt2 = &gaus_dico2_fx[i_mult2(ind2, step)];
    move16();

    FOR (i = 0; i < lg; i++)
    {
        /* code is Q9, Gaussian codebook is Q12 */
        /* code[i] = pt1[i] * sign1 + pt2[i] * sign2 */
        code[i] = add(mult(pt1[i], sign1), mult(pt2[i], sign2));
    }

    return;
}


/*-----------------------------------------------------*
 * dec_2pos()
 *
 * Decode the codevectors positions and signs
 *-----------------------------------------------------*/
static void dec_2pos_fx(
    Word16 index,  /* i  : quantization index      Q0  */
    Word16 *ind1,  /* o  : 1st vector index        Q0 */
    Word16 *ind2,  /* o  : 2nd vector index        Q0 */
    Word16 *sign1, /* o  : 1st vector sign         Q0 */
    Word16 *sign2, /* o  : 2nd vector sign         Q0 */
    Word16 log2_n  /* i  : Log2(number of vector)  Q0  */
)
{
    Word16 i;

    i = s_and(index, 1);
    *sign1 = (-32768); /* -1 (Q15) */           move16();
    if (i == 0)
    {
        *sign1 = MAX_16; /* 1 (Q15) */          move16();
    }
    *sign1 = shr(*sign1,3);
    move16();   /* To have code dec in Q9 instead of Q12 */

    index = shr(index, 1);

    *ind1 = shr(index, log2_n);
    move16();
    *ind2 = sub(index, shl(*ind1, log2_n));
    move16();
    *sign2 = *sign1;
    move16();
    if (sub(*ind1, *ind2) > 0)
    {
        *sign2 = negate(*sign1);
        move16();
    }
    return;
}



/*-----------------------------------------------------*
 * gaus_L2_dec :
 *
 * decoder of Gaussian Codebook for unvoiced as Layer 2
 *
 * One Gaussian vector
 *-----------------------------------------------------*/
void gaus_L2_dec(
    Word16 *code,         /* o  : decoded gaussian codevector     Q9  */
    Word16 tilt_code,     /* i  : tilt of code                    Q15 */
    const Word16 *A,      /* i  : quantized LPCs                  Q12 */
    Word16 formant_enh,   /* i  : formant enhancement factor      Q15 */
    Word16 *seed_acelp    /*i/o : random seed                     Q0  */
)
{
    Word16 i, seed;
    Word32 tmp32;

    /*Generate white gaussian noise using central limit theorem method (N only 4 as E_util_random is not purely uniform)*/
    seed = *seed_acelp;
    move16();
    FOR (i = 0; i < L_SUBFR; i++)
    {
        seed = own_random2_fx(seed);
        tmp32 = L_mac(0, seed, 1<<9);

        seed = own_random2_fx(seed);
        tmp32 = L_mac(tmp32, seed, 1<<9);

        seed = own_random2_fx(seed);
        code[i] = mac_r(tmp32, seed, 1<<9);
        move16();
    }
    *seed_acelp = seed;
    move16();

    /*Shape the gaussian excitation*/
    E_UTIL_cb_shape( 1, 0, 0, 1, 0, formant_enh, FORMANT_SHARPENING_G2, A, code, tilt_code, 0 );


    return;
}

