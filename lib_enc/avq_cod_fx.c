/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "prot_fx.h"        /* Function prototypes                    */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "stl.h"

/*-------------------------------------------------------------------*
* Function AVQ_cod()                                                *
*                                                                   *
* Split algevraic vector quantizer (AVQ) base on RE8 latice         *
*-------------------------------------------------------------------*/

void AVQ_cod_fx(           /* o:   comfort noise gain factor        */
    const Word16 xri[],      /* i:   vector to quantize               */
    Word16 xriq[],     /* o:   quantized normalized vector (assuming the bit budget is enough) */
    const Word16 NB_BITS,    /* i:   number of allocated bits         */
    const Word16 Nsv         /* i:   number of subvectors (lg=Nsv*8)  */
    ,const Word16 Q_in       /* i:   Scaling input    */
)
{
    Word16 i, l, iter, c[8];
    Word16 gain_inv, tmp, nbits, nbits_max, fac, offset;
    Word16 ebits[NSV_MAX], e_ebits, f_ebits, e_tmp,f_tmp, tmp16, l_8;
    Word32 Lener, Ltmp, Lgain, x1[8];

    /* find energy of each subvector in log domain (scaled for bits estimation) */
    FOR (l=0; l<Nsv; l++)
    {
        Lener = L_shl(4,shl(Q_in,1)); /* to set ebits >= 0 */
        FOR (i=0; i<8; i++)
        {
            Lener = L_mac(Lener, xri[l*8+i], xri[l*8+i]);
        }
        /* estimated bit consumption when gain=1 */
        /* ebits[l] = 5.0 * FAC_LOG2 * (Word16)log10(ener * 0.5) */
        e_ebits = norm_l(Lener);
        f_ebits = Log2_norm_lc(L_shl(Lener, e_ebits));
        e_ebits = sub(30-2, e_ebits);            /* -2 = *0.25 */
        e_ebits = sub(e_ebits, shl(Q_in,1));

        Ltmp = L_deposit_h(e_ebits);
        Ltmp = L_mac(Ltmp, f_ebits, 1);
        Ltmp = L_add(L_shl(Ltmp,6), L_shl(Ltmp,4)); /* Mult by 5.0 and then by 16 (To go to Q4). Do it using Mult by 80 (which is 64+16) */
        ebits[l] = round_fx(Ltmp);   /*Q4*/
    }
    /*----------------------------------------------------------------*
     * subvector energy worst case:
     * - typically, it's a tone with maximum of amplitude (RMS=23170).
     * - fft length max = 1024 (N/2 is 512)
     * log10(energy) = log10(23710*23710*1024*(N/2)) = 14.45
     * ebits --> 5.0*FAC_LOG2*14.45 = 240 bits
     *----------------------------------------------------------------*/

    /* estimate gain according to number of bits allowed */
    /* start at the middle (offset range = 0 to 255.75) Q6 */
    fac = 2048;
    move16();
    offset = 0;
    move16();

    Ltmp = L_mult(31130, sub(NB_BITS, Nsv));  /* (1810 - 8 - 1152/8)*.95*/
    nbits_max = round_fx(L_shl(Ltmp, 4));

    /* tree search with 10 iterations : offset with step of 0.25 bits (0.3 dB) */
    FOR (iter=0; iter<10; iter++)
    {
        offset = add(fac, offset);
        /* calculate the required number of bits */
        nbits = 0;
        move16();
        FOR (l=0; l<Nsv; l++)
        {
            tmp = sub(ebits[l], offset);
            tmp = s_max(tmp, 0);
            nbits = add(tmp, nbits);
        }
        /* decrease gain when no overflow occurs */
        if (sub(nbits, nbits_max) <= 0)
        {
            offset = sub(offset, fac);
        }
        fac = mult(fac, 16384);
    }

    Ltmp = L_shr(L_mult(offset, 13107), 6); /* offset((2^21)/160 */

    /* estimated gain (when offset=0, estimated gain=1) */
    f_tmp = L_Extract_lc(Ltmp, &e_tmp);
    tmp16 = extract_l(Pow2(14, f_tmp));
    Lgain = L_shl(tmp16, e_tmp);
    /* gain_inv = 1.0f / gain */
    e_tmp = norm_l(Lgain);
    tmp16 = extract_h(L_shl(Lgain, e_tmp));
    e_tmp = sub(31-14, e_tmp);
    gain_inv = div_s(16384, tmp16);
    e_tmp = sub(0, e_tmp);
    e_tmp = sub(e_tmp, Q_in);
    /* quantize all subvector using estimated gain */
    FOR (l=0; l<Nsv; l++)
    {
        l_8 = shl(l,3);
        FOR (i=0; i<8; i++)
        {
            x1[i] = L_shl(L_mult(xri[l_8+i], gain_inv), e_tmp);
            move32();
        }

        re8_PPV_fx(x1, c);

        FOR (i=0; i<8; i++)
        {
            xriq[l_8+i] = c[i];
            move16();
        }
    }

    /* round_fx bit allocations and save */
    FOR (i=0; i<Nsv; i++)
    {
        xriq[(Nsv*8)+i] = shl(ebits[i], 7-4);
        move16();
    }

    return;

}


/*-----------------------------------------------------------------*
* AVQ_encmux()
*
* Encode subvectors and write indexes into the bitstream
*-----------------------------------------------------------------*/
void AVQ_encmux_fx(
    Encoder_State_fx *st_fx,        /* i/o: encoder state structure      */
    const Word16 extl,       /* i  : extension layer                                 */
    Word16 xriq[],     /* i/o: rounded subvectors [0..8*Nsv-1] followed
                                    by rounded bit allocations [8*Nsv..8*Nsv+Nsv-1] */
    Word16 *nb_bits,   /* i/o: number of allocated bits                        */
    const Word16 Nsv,        /* i:   number of subvectors                            */
    Word16 nq_out[]    /* o  : AVQ nq index                                    */
)
{
    Word16 i, j=0, bits, pos, pos_max, overflow, pos_tmp;
    Word16 sort_idx[NSV_MAX], nq[NSV_MAX], kv[NSV_MAX*8];
    Word16 *t;
    UWord16  I[NSV_MAX];
    Word16  nq_ind, i_ind, kv_ind;

    test();
    IF( sub(extl,SWB_BWE_HIGHRATE) == 0 || sub(extl,FB_BWE_HIGHRATE) == 0 )
    {
        nq_ind = IND_NQ2;
        move16();
        i_ind = IND_I2;
        move16();
        kv_ind = IND_KV2;
        move16();
    }
    ELSE
    {
        nq_ind = IND_NQ;
        move16();
        i_ind = IND_I;
        move16();
        kv_ind = IND_KV;
        move16();
    }

    FOR(i = 0; i <NSV_MAX; i++)
    {
        I[i] = (UWord16)-1;
        move16();
    }

    /*-----------------------------------------------------------------
     * Encode subvectors and fix possible overflows in total bit budget,
     * i.e. find for each subvector a codebook index nq (nq=0,2,3,4,...,NSV_MAX),
     * a base codebook index (I), and a Voronoi index (kv)
     *-----------------------------------------------------------------*/

    /* sort subvectors by estimated bit allocations in decreasing order */
    t = kv;
    move16();   /* reuse vector to save memory */
    move16();  /*ptr init*/
    FOR( i=0; i<Nsv; i++ )
    {
        t[i] = xriq[8*Nsv+i];
        move16();
    }

    FOR( i=0; i<Nsv; i++ )
    {
        bits = t[0];
        move16();
        pos = 0;
        move16();
        FOR( j=1; j<Nsv; j++ )
        {
            if( sub(t[j],bits) > 0 )
            {
                pos = j;
                move16();
            }
            bits = s_max(t[j],bits);

        }
        sort_idx[i] = pos;
        move16();
        t[pos] = -1;
        move16();
    }

    /* compute multi-rate indices and avoid bit budget overflow */
    pos_max = 0;
    move16();
    bits = 0;
    move16();
    FOR( i=0; i<Nsv; i++ )
    {
        /* find vector to quantize (criteria: nb of estimated bits) */
        pos = sort_idx[i];
        move16();

        /* compute multi-rate index of rounded subvector (nq,I,kv[]) */
        re8_cod_fx( &xriq[pos*8], &nq[pos], &I[pos], &kv[8*pos] );

        IF( nq[pos] > 0 )
        {
            j = pos_max;
            move16();
            j = s_max(pos,j);

            /* compute (number of bits -1) to describe Q #nq */
            IF(sub(nq[pos],2) >= 0 )
            {
                overflow = sub(i_mult2(nq[pos],5),1);
            }
            ELSE
            {
                overflow = 0;
                move16();
            }

            /* check for overflow and compute number of bits-1 (n) */
            IF( sub(add(bits,add(overflow,j)),*nb_bits) > 0 )
            {
                /* if budget overflow */
                pos_tmp = add(shl(pos,3),8);  /*(pos*8)+8*/
                FOR( j=pos*8; j<pos_tmp; j++ )
                {
                    xriq[j] = 0;
                    move16();
                }
                nq[pos] = 0;
                move16();/* force Q0 */
            }
            ELSE
            {
                bits = add(bits, overflow);
                pos_max = j;
                move16();/* update index of the last described subvector */
            }
        }
    }

    /* write indexes to the bitstream */
    /* ============================== */

    bits = *nb_bits;
    move16();
    overflow = 0;
    move16();

    FOR( i=0; i<Nsv; i++ )
    {
        if( sub(sub(i_mult2(5,nq[i]),1),bits) == 0) /* check the overflow */
        {
            overflow = 1;
            move16();
        }

        IF( sub(bits,8) > 0 )
        {
            /* write the unary code for nq[i] */
            j = sub(nq[i], 1);
            IF ( nq[i] > 0 )
            {
                /* write the unary code */
                FOR( ; j > 16; j -= 16 )
                {
                    push_indice_fx( st_fx, nq_ind, 65535, 16 );
                    bits = sub(bits,16);
                }

                IF ( j > 0 )
                {
                    push_indice_fx( st_fx, nq_ind, extract_l(L_sub(L_shl(1L,j),1L)), j );
                    bits = sub(bits,j);
                }
            }
            IF ( !overflow )
            {
                /* write the stop bit */
                push_indice_fx( st_fx, nq_ind, 0, 1 );
                bits = sub(bits,1);
            }

            /* write codebook indices (rank I and event. Voronoi index kv) */
            IF( nq[i] == 0 )    /* Q0 */
            {
                /* nothing to write */
            }
            ELSE IF( sub(nq[i],5) < 0 )    /* Q2, Q3, Q4 */
            {
                push_indice_fx( st_fx, i_ind, I[i], shl(nq[i],2) );
                bits = sub(bits, shl(nq[i],2));
            }
            ELSE IF( s_and(nq[i],1) == 0 )    /* Q4 + Voronoi extensions r=1,2,3,... */
            {
                push_indice_fx( st_fx, i_ind, I[i], 4*4 );
                bits = sub(bits,4*4);
                pos = sub(shr(nq[i],1),  2);  /* Voronoi order determination */
                move16();   /*ptr init*/
                FOR( j=0; j<8; j++ )
                {
                    push_indice_fx( st_fx, kv_ind, kv[i*8+j], pos );
                }

                bits = sub(bits,shl(pos,3));
            }
            ELSE    /* Q3 + Voronoi extensions r=1,2,3,... */
            {
                push_indice_fx( st_fx, i_ind, I[i], 4*3 );
                bits = sub(bits, 4*3);

                pos = sub(shr(nq[i],1), 1);  /* Voronoi order determination */
                move16();  /* ptr init */
                FOR( j=0; j<8; j++ )
                {
                    push_indice_fx( st_fx, kv_ind, kv[i*8+j], pos );
                }

                bits = sub(bits,shl(pos,3));
            }
        }
    } /* for */

    *nb_bits = bits;
    move16();

    FOR( i=0; i<Nsv; i++ )
    {
        nq_out[i] = nq[i];
        move16();
    }

    return;
}


/*-------------------------------------------------------------------*
* Function AVQ_cod_lpc()                                            *
*                                                                   *
* Split algebraic vector quantizer (AVQ) for LPC quantization       *
*-------------------------------------------------------------------*/

void AVQ_cod_lpc(
    Word16 *nvec,  /* input:  vector to quantize (normalized) (5Q10)*/
    Word16 *nvecq, /* output: quantized vector                (5Q10)*/
    Word16 *indx,  /* output: index[] (4 bits per words)      (15Q0)*/
    Word16 Nsv     /* input:  number of subvectors (lg=Nsv*8)       */
)
{
    Word16	ival, n, nq, nk, c[8], kv[8];
    Word16	i, l, pos;
    Word32  I;
    Word32 x1[8];
    UWord16 I16;


    /* quantize all subvector using estimated gain */
    pos = Nsv;
    move16();
    FOR (l=0; l<Nsv; l++)
    {
        FOR (i=0; i<8; i++)
        {
            x1[i] = L_mult(nvec[l*8+i], 1<<4);  /* 5Q10 -> 16Q15*/ move32();
        }
        re8_PPV_fx(x1, c);				    /*x1:8Q15, c:15Q0*/
        re8_cod_fx(c, &nq, &I16, kv);
        I = UL_deposit_l(I16);

        FOR (i=0; i<8; i++)
        {
            nvecq[l*8+i] = shl(c[i],10);    /*15Q0->5Q10*/						   move16();
        }

        indx[l] = nq;      /* index[0..Nsv-1] = quantizer number (0,2,3,4...) */   move16();
        nk = 0;
        move16();
        n = nq;
        move16();

        IF (sub(nq,4) > 0)
        {
            nk = shr(sub(nq,3),1);			/*nk = (nq-3)>>1;*/
            n = sub(nq,shl(nk,1));			/*n = nq - nk*2; */
        }

        /* write n groups of 4-bit for base codebook index (I) */
        FOR ( ; n > 0; n-- )
        {
            indx[pos++] = s_and(extract_l(I),0x000F);
            move16();
            I = L_shr(I,4);
        }

        /* write n groups of 4-bit for Voronoi index (k[]) */
        FOR ( ; nk > 0; nk--)
        {
            ival = 0;
            move16();
            FOR (i=0; i<8; i++)
            {
                ival = shl(ival,1);							/*ival <<= 1;*/
                ival = add(ival,s_and(kv[i],0x0001));		/*ival += (kv[i] & 0x01);*/
                kv[i] = shr(kv[i],1);						/*kv[i] >>= 1;*/		move16();
            }
            indx[pos++] = s_and(ival,0x000F);
            move16();
            ival = shr(ival,4);
            indx[pos++] = s_and(ival,0x000F);
            move16();
        }
    }


    return;
}
