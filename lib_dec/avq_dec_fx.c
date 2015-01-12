/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"      /* Compilation switches                   */
#include "cnst_fx.h"      /* Common constants                       */
#include "rom_com_fx.h"   /* Static table prototypes                */
#include "prot_fx.h"      /* Function prototypes                    */
#include "stl.h"

/*-----------------------------------------------------------------*
 *   Function  AVQ_Demuxdec_Bstr                                   *
 *            ~~~~~~~~~~~~~~~~~~                                   *
 *   Read indexes from one bitstream and decode subvectors.        *
 *-----------------------------------------------------------------*/

void AVQ_demuxdec_fx(
    Decoder_State_fx *st_fx,      /* i/o: decoder state structure */
    Word16  xriq[],   /* o:   decoded subvectors [0..8*Nsv-1] */
    Word16  *nb_bits, /* i/o: number of allocated bits        */
    const Word16  Nsv,      /* i:   number of subvectors            */
    Word16 nq_out[]   /* i/o: AVQ nq index                    */
)
{
    Word16 i,j, bits, order_v;
    UWord16 I[NSV_MAX];
    Word16 nq[NSV_MAX], *kv, code[8];
    Word16 tmp16;

    set16_fx( (Word16*)I, 0, NSV_MAX );
    set16_fx( code, 0, 8 );

    kv = xriq;        /* reuse vector to save memory */
    bits = *nb_bits;
    move16();

    FOR( i=0; i<Nsv; i++ )
    {
        nq[i] = 0;
        move16();/* initialization and also forced if the budget is exceeded */

        IF( sub(bits, 8) > 0 )
        {
            /* read the unary code including the stop bit for nq[i] */
            nq[i] = -1;
            move16();
            tmp16 = 0;
            move16();
            DO
            {
                nq[i] = add(nq[i], 1);
                move16();

                tmp16 = extract_l(L_mac0(4,nq[i],5));
                tmp16 = sub(tmp16,bits);
                IF( tmp16 == 0 )
                {
                    BREAK;
                }
            }
            WHILE( get_next_indice_1_fx(st_fx) != 0 );

            if( tmp16 == 0 ) /* check the overflow */
            {
                bits = add(bits,1);  /* overflow stop bit */
            }

            bits = sub(bits,nq[i]);
            bits = sub(bits,1);         /* count the stop bit */

            if( nq[i] > 0 )
            {
                nq[i] = add(nq[i],1);
                move16();
            }

            /* read codebook indices (rank I and event. Voronoi index kv) */
            IF( nq[i] != 0 )    /* for Q0 nothing to read */
            {
                IF( sub(nq[i], 5) < 0 )    /* Q2, Q3, Q4 */
                {
                    tmp16 = shl(nq[i], 2);
                    order_v = 0;
                    move16();
                }
                ELSE            /* for Q3/Q4 + Voronoi extensions r=1,2 */
                {
                    j = 1;
                    move16();
                    if( s_and(nq[i], 1) == 0 )
                    {
                        j = add(j,1);
                    }
                    order_v = sub(shr(nq[i], 1), j);  /* Voronoi order determination */
                    tmp16 = shl(add(j, 2), 2);
                }

                I[i] = get_next_indice_fx(st_fx, tmp16 );
                move16();
                bits = sub(bits, tmp16);

                IF( order_v > 0 )
                {
                    tmp16 = shl(i, 3);
                    FOR( j=0; j<8; j++ )
                    {
                        kv[tmp16+j] = (Word16)get_next_indice_fx(st_fx, order_v );
                        move16();
                    }
                    bits = sub(bits, shl(order_v, 3));
                }
            }
        }
    }

    /* decode all subvectors */
    FOR( i=0; i<Nsv; i++ )
    {
        /* multi-rate RE8 decoder */
        re8_dec_fx( nq[i], I[i], kv, code );
        kv += 8;

        /* write decoded RE8 vector to decoded subvector #i */
        Copy( code, xriq, 8 );
        xriq += 8;
    }

    *nb_bits = bits;
    move16();

    FOR ( i=0; i<Nsv; i++ )
    {
        nq_out[i] = nq[i];
        move16();
    }
    return;
}



/*-----------------------------------------------------------------*
 * AVQ_dec_lpc()
 *
 * Demultiplex and decode subvectors for LPC dequantization
 * using split algebraic vector dequantizer
 *-----------------------------------------------------------------*/

void AVQ_dec_lpc(
    Word16 *indx,    /* input:  index[] (4 bits per words)      */
    Word16 *nvecq,   /* output: vector quantized                */
    Word16 Nsv       /* input:  number of subvectors (lg=Nsv*8) */
)
{
    Word16  i, l, n, nq, nk, pos, ival, c[8], kv[8];
    Word32  I;
    UWord16 I16;

    /* last index word */
    pos = sub(Nsv, 1);

    FOR (l=0; l < Nsv; l++)
    {
        pos = add(pos, indx[l]);
    }

    /* decode all subvectors */

    FOR (l=Nsv-1; l>=0; l--)
    {
        nq = indx[l];        /* quantizer number (0,2,3..n) */  move16();

        nk = 0;
        move16();
        n = nq;
        move16();

        IF (sub(nq, 4) > 0)
        {
            nk = shr(sub(nq, 3), 1);
            n = sub(nq, shl(nk, 1));
        }

        /* read n groups of 4-bit for Voronoi index (k[]) */

        FOR (i=0; i<8; i++)
        {
            kv[i] = 0;
            move16();
        }

        FOR ( ; nk > 0; nk--)
        {
            ival = s_and(indx[pos--], 0x0F);
            ival = shl(ival, 4);
            ival = add(ival, s_and(indx[pos--], 0x0F));

            FOR (i=7; i>=0; i--)
            {
                kv[i] = shl(kv[i], 1);
                kv[i] = add(kv[i], s_and(ival, 0x01));
                move16();
                ival = shr(ival, 1);
            }
        }

        /* read n groups of 4-bit for base codebook index (I) */
        I = L_deposit_l(0);
        FOR (; n > 0; n--)
        {
            I = L_shl(I, 4);
            I = L_add(I, (Word32) s_and(indx[pos--], 0x0F));
        }

        /* multi-rate RE8 decoder */
        I16 = (UWord16)extract_l(I);
        cast16();
        re8_dec_fx(nq, I16, kv, c);

        /* write decoded RE8 vector */
        FOR (i=0; i<8; i++)
        {
            nvecq[(l*8)+i] = c[i];
            move16();
        }
    }

    return;
}
