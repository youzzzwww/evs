/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "prot_fx.h"       /* Function prototypes                    */
#include "assert.h"      /* Debug prototypes                       */
#include "stl.h"

/*==================================================================================*/
/* FUNCTION : void bands_and_bit_alloc_fx();							            */
/*----------------------------------------------------------------------------------*/
/* PURPOSE  :  AC mode (GSC) bands and bits allocation                              */
/*----------------------------------------------------------------------------------*/
/*    INPUT ARGUMENTS :												                */
/* _ (Word16) cor_strong_limit   : HF correlation 				                    */
/* _ (Word16) noise_lev          : dwn scaling factor				     Q0         */
/* _ (Word32) core_brate         : core codec used 						 Q0         */
/* _ (Word16) Diff_len           : Lenght of the difference signal		 Q0	        */
/* _ (Word16) bits_used          : Number of bit used before frequency   Q0         */
/* _ (Word16) idx                : Energy band 14 					     Q0         */
/* _ (Word16*) exc_diff          : Difference signal to quantize (Encoder only)	    */
/* _ (Word16) coder_type         : coding type      					 Q0         */
/* _ (Word16) bwidth             : input signal bandwidth				 Q0         */
/*----------------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													            */
/* _ (Word16*) max_ener_band     : Sorted order      					            */
/* _ (Word16*) nb_subbands       : Number of subband allowed    		Q0          */
/* _ (Word16*) concat_in         :  Concatened PVQ's input vector  (Encoder Only)   */
/* _ (Word16*) pvq_len           : Number of bin covered with the PVQ 	Q0          */
/*----------------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :  											            */
/* _ (Word16*) bit			   :Number of bit allowed for frequency quantization	*/
/* _ (Word16*) Ener_per_bd_iQ  : Quantized energy vector                        Q13 */
/* _ (Word32*) bits_per_bands  : Number of bit allowed per allowed subband      Q18 */
/*----------------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													            */
/* _ None																            */
/*==================================================================================*/
void bands_and_bit_alloc_fx(
    const Word16 cor_strong_limit, /* i  : HF correlation                                        */
    const Word16 noise_lev,        /* i  : dwn scaling factor                                    */
    const Word32 core_brate,       /* i  : core bit rate                                         */
    const Word16 Diff_len,         /* i  : Lenght of the difference signal (before pure spectral)*/
    const Word16 bits_used,        /* i  : Number of bit used before frequency Q                 */
    Word16 *bit,             /* i/o: Number of bit allowed for frequency quantization      */
    const Word16 *Ener_per_bd_iQ,  /* i/o: Quantized energy vector                               */
    Word16 *max_ener_band,   /* o  : Sorted order                                          */
    Word16 *out_bits_per_bands,  /* i/o: Number of bit allowed per allowed subband        Q3     */
    Word16 *nb_subbands,     /* o  : Number of subband allowed                             */
    const Word16 *exc_diff,        /* i  : Difference signal to quantize (encoder side only)     */
    Word16 *concat_in,       /* o  : Concatened PVQ's input vector (encoder side only)     */
    Word16 *pvq_len,         /* o  : Number of bin covered with the PVQ                    */
    const Word16 coder_type,       /* i  : coding type                                           */
    const Word16 bwidth,           /* i  : input signal bandwidth                                */
    const Word16 GSC_noisy_speech
)
{

    Word16 bandoffset, i, j, nb_bands_max, bit_new_bands, bit_tmp, st_band, nb_bands;
    Word16 ener_vec[MBANDS_GN];   /*Q12 */
    Word16 nb_tot_bands = 16;
    Word16 bit_index, bit_index_mem, imax;
    Word32 L_tmp;
    Word32 sum_bit, bit_fracf;
    Word16 etmp;
    Word16 tmp;
    Word16 Ener_per_bd_iQ_tmp[MBANDS_GN];
    Word16 pos, band;
    Word16 SWB_bit_budget;
    Word32 bits_per_bands[MBANDS_GN];
    Word16 w_sum_bit;

    Copy( Ener_per_bd_iQ, Ener_per_bd_iQ_tmp, MBANDS_GN );

    set32_fx( bits_per_bands, 0, MBANDS_GN );
    set16_fx( out_bits_per_bands, 0, MBANDS_GN );

    /* To adapt current energy band to PVQ freq band for sorting*/
    ener_vec[0] = add(Ener_per_bd_iQ[0],Ener_per_bd_iQ[1]);   /*Q12 */
    Copy( Ener_per_bd_iQ_tmp+1, ener_vec, 15 );  /*Q12 */
    ener_vec[15] = ener_vec[14];
    move16();

    /*------------------------------------------------------------------------
     * Determination of the number of bits available to the frequency domain
     * Allocation of a maximum number of band to be encoded
     *-----------------------------------------------------------------------*/

    nb_bands_max = nb_tot_bands;
    move16();
    bit_new_bands = 5;
    move16();

    bit_index = i_mult2(BRATE2IDX_fx(core_brate),17);
    bit_index_mem = bit_index;
    move16();

    test();
    test();
    IF( (sub(coder_type,AUDIO) == 0 || sub(coder_type,INACTIVE) == 0) && sub(bwidth,NB) == 0 )
    {
        IF(L_sub(core_brate,ACELP_9k60) >= 0)
        {
            /*  *bit = (short)(core_brate*(1.0f/50) + 0.5f) - bits_used - 25; */
            L_tmp =  Mult_32_16(core_brate,20971);
            tmp = extract_l(L_shr_r(L_tmp,5));
            *bit = sub(sub(tmp,bits_used), 25);
            move16();
        }
        ELSE
        {
            L_tmp =  Mult_32_16(core_brate,20971);
            tmp = extract_l(L_shr_r(L_tmp,5));
            *bit = sub(sub(tmp,bits_used), 21);
            move16();
        }
        nb_tot_bands = 10;
        move16();
    }
    ELSE
    {
        /* *bit = (short)(core_brate*(1.0f/50) + 0.5f) - bits_used - GSC_freq_bits[bit_index]; */

        L_tmp =  Mult_32_16(core_brate,20971);
        tmp = extract_l(L_shr_r(L_tmp,5));
        *bit = sub(sub(tmp,bits_used),GSC_freq_bits[bit_index]);
        move16();
    }

    IF( sub(GSC_noisy_speech,1) == 0 )
    {
        SWB_bit_budget = *bit;
        move16();
        nb_bands = 5;
        move16();
        st_band = nb_bands;
        move16();

        set32_fx( bits_per_bands, 0, MBANDS_GN );
        /*bit_fracf = (1.0f/nb_bands)*(SWB_bit_budget); */
        bit_fracf = L_mult(div_s(1,nb_bands),shl(SWB_bit_budget,2));	 /* Q18  */

        nb_tot_bands = sub(nb_bands_max,6);
        nb_tot_bands = s_min(nb_tot_bands, 16);

        FOR(j = 0; j < 2; j++)
        {
            i = j;
            move16();
            max_ener_band[j] = i;
            move16();
            ener_vec[i] = 0;
            move16();
        }

        FOR(; j < nb_bands; j++)
        {
            i = maximum_fx(ener_vec, nb_tot_bands, &etmp);
            max_ener_band[j] = i;
            move16();
            ener_vec[i] = 0;
            move16();
        }

        set32_fx(bits_per_bands, bit_fracf, nb_bands);
    }
    ELSE
    {
        bit_index++;
        bit_tmp = sub(*bit,GSC_freq_bits[bit_index]);
        bit_index++;
        nb_bands_max = add(nb_bands_max,GSC_freq_bits[bit_index]);
        bit_index++;

        *pvq_len = 112;
        move16();
        st_band = 7;
        move16();

        IF( L_sub(core_brate,ACELP_9k60) <= 0 )
        {
            *pvq_len = 80;
            move16();
            st_band = 5;
            move16();

            IF( Diff_len == 0 )
            {
                nb_bands_max = add(nb_bands_max,2);
                bit_tmp = sub(bit_tmp,13);
            }
        }

        ELSE IF( Diff_len == 0 )
        {
            nb_bands_max = add(nb_bands_max,2);
            bit_tmp = sub(bit_tmp,17);
        }

        nb_bands = shr(*pvq_len,4);

        /*------------------------------------------------------------------------
         * Ajustement of the maximum number of bands in function of the
         * dynamics of the spectrum (more or less speech like)
         *-----------------------------------------------------------------------*/
        test();
        test();
        test();
        test();
        IF( sub(coder_type,INACTIVE) == 0 || sub(noise_lev,NOISE_LEVEL_SP3) >= 0 )
        {
            /* Probably classification error -> concentrate bits on LF */
            nb_bands_max = nb_bands;
            move16();
            if( L_sub(core_brate,ACELP_8k00) >= 0 )
            {
                nb_bands_max = add(nb_bands,1);
            }
        }
        ELSE IF( sub(noise_lev,NOISE_LEVEL_SP2) >= 0 ||
                 (L_sub(core_brate,ACELP_13k20) <= 0 && L_sub(core_brate,ACELP_9k60) >= 0 && cor_strong_limit == 0) )  /* Very low dynamic, tend to speech, do not try to code HF at all */
        {
            nb_bands_max = sub(nb_bands_max,2);
        }
        ELSE if( sub(noise_lev,NOISE_LEVEL_SP1) >= 0) /* Very low dynamic, tend to speech, code less HF */
        {
            nb_bands_max = sub(nb_bands_max,1);
        }

        test();
        if( sub(bwidth,NB) == 0 && sub(nb_bands_max,10) > 0 )
        {
            nb_bands_max = 10;
            move16();
        }

        /*------------------------------------------------------------------------
         * Find extra number of band to code according to bit rate availables
         *-----------------------------------------------------------------------*/
        WHILE ( sub(bit_tmp,bit_new_bands) >= 0 && sub(nb_bands,sub(nb_bands_max, 1)) <= 0 )
        {
            bit_tmp = sub(bit_tmp,bit_new_bands);
            nb_bands = add(nb_bands,1);
        }

        /*------------------------------------------------------------------------
         * Fractional bits to distribute on the first x bands
         *-----------------------------------------------------------------------*/

        bit_fracf = L_mult(div_s(1,st_band),shl(bit_tmp,2));	 /* Q18  */

        /*------------------------------------------------------------------------
         * Complete the bit allocation per frequency band
         *-----------------------------------------------------------------------*/
        imax = 5;
        move16();

        if( L_sub(core_brate,ACELP_9k60) > 0 )
        {
            imax = 7;
            move16();
        }
        FOR(i = 0; i < imax; i++)
        {
            bits_per_bands[i] = L_add(GSC_freq_bits_fx[bit_index],bit_fracf);
            move32();/* Q18 */
            bit_index = add(bit_index,1);
        }

        IF( Diff_len == 0 )
        {
            bit_index = add(bit_index_mem,10);
            FOR( i = 0; i < 7; i++ )
            {
                bits_per_bands[i] = L_add(bits_per_bands[i],GSC_freq_bits_fx[bit_index]);
                move32();/*chk Q18 */
                bit_index = add(bit_index,1);
            }
        }

        /*--------------------------------------------------------------------------
         * Complete the bit allocation per frequency band for 16kHz high brate mode
         *--------------------------------------------------------------------------*/

        FOR( j = st_band; j < nb_bands; j++ )
        {
            bits_per_bands[j] = L_shl(bit_new_bands,18);
            move32(); /*chk Q18 */
        }

        /*--------------------------------------------------------------------------
         * Compute a maximum band (band offset) for the search on maximal energy
         * This is function of the spectral dynamic and the bitrate
         *--------------------------------------------------------------------------*/

        bandoffset = sub(nb_tot_bands,add(nb_bands,2));

        test();
        test();
        test();
        test();
        test();
        IF( sub(noise_lev,NOISE_LEVEL_SP1a) <= 0 )
        {
            bandoffset = sub(bandoffset,1);
        }
        ELSE if ( (L_sub(core_brate,ACELP_13k20) <= 0 && (sub(coder_type,INACTIVE) == 0 || sub(noise_lev,NOISE_LEVEL_SP3) >= 0)) ||
                  (L_sub(core_brate,ACELP_13k20) <= 0 && L_sub(core_brate,ACELP_9k60) >= 0 && cor_strong_limit == 0) )
        {
            bandoffset = add(bandoffset,1);
        }

        bandoffset = s_max(bandoffset ,0);

        /*--------------------------------------------------------------------------
         * Initiazed sorted vector
         * For the first x bands to be included in th final sorted vector
         * Sort the remaining bands in decrease energy order
         *--------------------------------------------------------------------------*/
        FOR(j = 0; j < nb_tot_bands; j++)
        {
            max_ener_band[j] = -10;
            move16();
        }
        FOR(j = 0; j < st_band; j++)
        {
            max_ener_band[j] = j;
            move16();
            ener_vec[j] = -10;
            move16();
        }
        pos = st_band;
        move16();
        FOR(; j < nb_bands; j++)
        {
            i = maximum_fx(ener_vec, sub(nb_tot_bands,bandoffset), &etmp);
            pos = s_max(pos,i);
            max_ener_band[j] = i;
            move16();
            ener_vec[i] = -10;
            move16();
        }

        /* re-allocate bits to the frames such that the highest band with allocated bits is higher than the threshold */
        test();
        test();
        test();
        IF( sub(sub(nb_tot_bands, bandoffset),nb_bands) > 0  && ( sub(pos,7) > 0 && L_sub(core_brate,ACELP_8k00) == 0 ) && sub(bwidth,WB) == 0 )
        {
            band = sub(nb_tot_bands, add(bandoffset,nb_bands));
            FOR(j=0; j<band; j++)
            {
                i = maximum_fx( ener_vec, sub(nb_tot_bands,bandoffset), &etmp );
                max_ener_band[add(nb_bands,j)] = i;
                move16();
                ener_vec[i] = -10;
                move16();
                bits_per_bands[add(nb_bands,j)] = 1310720;
                move32(); /*Q18 */
            }
            nb_bands = add(nb_bands,band);

            bit_tmp = i_mult2(band,5);

            IF( sub(band,2) <= 0 )
            {
                FOR(j = sub(st_band,1); j < nb_bands; j++)
                {
                    bits_per_bands[j] = L_add(bits_per_bands[j],262144); /*Q18 */ move32();
                }
                bit_tmp = add(bit_tmp, add(sub(nb_bands, st_band) , 1));
            }

            i = 0;
            move16();
            j = 0;
            move16();
            FOR( ; bit_tmp > 0;  bit_tmp--)
            {
                bits_per_bands[j] = L_sub(bits_per_bands[j],262144); /*Q18 */
                j = add(j,1);
                if ( sub(j,sub(st_band, i)) == 0 )
                {
                    j = 0;
                    move16();
                }
                test();
                if( j == 0 && sub(i,sub(st_band, 1)) < 0)
                {
                    i = add(i,1);
                }
            }
        }
    }
    /*--------------------------------------------------------------------------
     * Bit sum verification for GSC inactive at very high rate
     * The maximum number of bits per band of length 16 is 112
     * Redistribute the overage bits if needed
     *--------------------------------------------------------------------------*/
    sum_bit = 0;
    move16();
    j = 0;
    move16();
    FOR( i = 0; i < nb_bands; i++ )
    {
        L_tmp = Mult_32_16(sum_bit,10923);

        IF( L_sub(bits_per_bands[i],29360128) > 0)   /* 112 in Q18 */
        {
            sum_bit = L_add(sum_bit,L_sub(bits_per_bands[i],29360128)); /* Q18 */
            bits_per_bands[i] = 29360128;
            move32();
            j = add(i,1);
        }
        ELSE if( L_sub(L_add(bits_per_bands[i],L_tmp),29360128 ) > 0)  /* Q18 */
        {
            j = add(i,1);
        }
    }

    IF( sum_bit != 0 )
    {
        tmp = sub(nb_bands,j);
        sum_bit = Mult_32_16(sum_bit,div_s(1,tmp));   /* Q18 */
        FOR( i = j; i < nb_bands; i++ )
        {
            bits_per_bands[i] = L_add(bits_per_bands[i],sum_bit);
            move32();/* Q18 */
        }
    }
    /*--------------------------------------------------------------------------
     * second step of bit sum verification, normally sum_bit == *bit
     *--------------------------------------------------------------------------*/
    w_sum_bit = 0;
    move16();
    FOR( i = 0; i < nb_bands; i++ )
    {
        out_bits_per_bands[i] = shl(extract_l(L_shr(bits_per_bands[i],18)),3);
        move16();
        w_sum_bit = add(w_sum_bit,out_bits_per_bands[i]);  /* Q3 */
    }
    tmp = shl(*bit,3);

    IF(  sub(tmp,w_sum_bit)>0 )
    {
        i = sub(nb_bands,1);
        move16();
        FOR( ; tmp > w_sum_bit; w_sum_bit += (1<<3) )
        {
            out_bits_per_bands[i] = add(out_bits_per_bands[i],1<<3);
            move16();
            i = sub(i, 1);
            if(i==0)
            {
                i = sub(nb_bands,1);
            }
        }
    }
    /*--------------------------------------------------------------------------
     * Recompute the real number/length of frequency bands to encode
     *--------------------------------------------------------------------------*/
    *nb_subbands = nb_bands;
    move16();
    *pvq_len = shl(*nb_subbands,4);

    /*--------------------------------------------------------------------------
     * Concatenate bands (encoder only)
     *--------------------------------------------------------------------------*/
    IF( exc_diff != NULL )
    {
        FOR( j = 0; j < nb_bands; j++ )
        {
            Copy( exc_diff + shl(max_ener_band[j],4), concat_in+shl(j,4), 16 );
        }
    }

    return;
}
