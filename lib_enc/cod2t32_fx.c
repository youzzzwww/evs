/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "cnst_fx.h"        /* Common constants                       */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"


/*-------------------------------------------------------------------*
* Local Constants
*-------------------------------------------------------------------*/

#define STEP        2
#define MSIZE       1024

/*----------------------------------------------------------------------------------
* Function  acelp_2t32()
*
* 12 bits algebraic codebook.
* 2 tracks x 32 positions per track = 64 samples.
*
* 12 bits --> 2 pulses in a frame of 64 samples.
*
* All pulses can have two (2) possible amplitudes: +1 or -1.
* Each pulse can have 32 possible positions.
*----------------------------------------------------------------------------------*/

void acelp_2t32_fx(
    Encoder_State_fx *st_fx,       /* i/o: encoder state structure */
    const Word16 dn[],             /* i  : corr. between target and h[].                 */
    const Word16 h[],              /* i  : impulse response of weighted synthesis filter */
    Word16 code[],           /* o  : algebraic (fixed) codebook excitation         */
    Word16 y[]               /* o  : filtered fixed codebook excitation            */
)
{
    Word16 i, j, k, i0, i1, ix, iy, pos, pos2, sign0, sign1,index;
    Word16 ps1, ps2, alpk, alp1, alp2;
    Word16 sq,psk;
    Word16 pol[L_SUBFR], dn_p[L_SUBFR];
    Word16 ii,jj;
    Word16 *p0, *p1, *p2;
    const Word16 *ptr_h1, *ptr_h2, *ptr_hf;

    Word32 s, L_cor;
    Word32 L_tmp;
    Word16 rrixix[NB_TRACK_FCB_2T][NB_POS_FCB_2T];
    Word16 rrixiy[MSIZE];

    /*----------------------------------------------------------------*
     * Compute rrixix[][] needed for the codebook search.
     *----------------------------------------------------------------*/

    /* Init pointers to last position of rrixix[] */
    p0 = &rrixix[0][NB_POS_FCB_2T - 1];
    move16();
    p1 = &rrixix[1][NB_POS_FCB_2T - 1];
    move16();

    ptr_h1 = h;
    move16();
    L_cor = L_deposit_h(1);
    FOR (i = 0; i < NB_POS_FCB_2T; i++)
    {
        L_cor = L_mac(L_cor, *ptr_h1, *ptr_h1);
        ptr_h1++;
        *p1-- = extract_h(L_cor);
        move16();				/*Q9 Q7*/
        L_cor = L_mac(L_cor, *ptr_h1, *ptr_h1);
        ptr_h1++;
        *p0-- = extract_h(L_cor);
        move16();					/*Q9 Q7*/
    }

    p0 = rrixix[0];
    move16();
    p1 = rrixix[1];
    move16();

    FOR (i = 0; i < NB_POS_FCB_2T; i++)
    {
        *p0 = shr(*p0, 1);
        move16();
        p0++;
        *p1 = shr(*p1, 1);
        move16();
        p1++;
    }

    /*------------------------------------------------------------*
      * Compute rrixiy[][] needed for the codebook search.
      *------------------------------------------------------------*/

    pos = MSIZE - 1;
    move16();
    pos2 = MSIZE - 2;
    move16();
    ptr_hf = h + 1;
    move16();

    FOR (k = 0; k < NB_POS_FCB_2T; k++)
    {
        /* Init pointers to last position of diagonals */
        p1 = &rrixiy[pos];
        move16();
        p0 = &rrixiy[pos2];
        move16();

        ptr_h1 = h;
        move16();
        ptr_h2 = ptr_hf;
        move16();

        L_cor = L_mult(*ptr_h1++, *ptr_h2++);
        FOR (i = k; i < NB_POS_FCB_2T-1; i++)
        {
            *p1 = round_fx(L_cor);
            L_cor = L_mac(L_cor, *ptr_h1++, *ptr_h2++);
            *p0 = round_fx(L_cor);
            L_cor = L_mac(L_cor, *ptr_h1++, *ptr_h2++);

            p1 -= (NB_POS_FCB_2T + 1);
            move16();
            p0 -= (NB_POS_FCB_2T + 1);
            move16();
        }

        *p1 = round_fx(L_cor);

        pos -= NB_POS_FCB_2T;
        move16();
        pos2--;
        ptr_hf += STEP;
        move16();
    }

    /*----------------------------------------------------------------*
     * computing reference vector and pre-selection of polarities
     *----------------------------------------------------------------*/

    L_tmp = L_deposit_h(dn[0]);
    FOR(i=0; i<L_SUBFR; i++)
    {
        /* FIR high-pass filtering */
        IF(i==0)
        {
            L_tmp = L_msu(L_tmp, dn[1],11469);
        }
        ELSE IF(sub(i,L_SUBFR-1)==0)
        {
            L_tmp = L_deposit_h(dn[i]);
            L_tmp = L_msu(L_tmp,dn[i-1],11469);
        }
        ELSE
        {
            L_tmp = L_deposit_h(dn[i]);
            L_tmp = L_msu(L_tmp,dn[i-1],11469);
            L_tmp = L_msu(L_tmp,dn[i+1],11469);
        }

        /* pre-selection of polarities */
        IF(L_tmp>=0)
        {
            pol[i] = 1;
            move16();
            dn_p[i] = dn[i];
            move16();
        }
        ELSE
        {
            pol[i] = -1;
            move16();
            dn_p[i] = negate(dn[i]);
            move16();
        }
    }

    /*----------------------------------------------------------------*
     * compute denominator ( multiplied by polarity )
     *----------------------------------------------------------------*/

    k=0;
    ii=0;
    move16();
    move16();
    FOR(i=0; i<NB_POS_FCB_2T; i++)
    {
        jj=1;
        move16();
        FOR(j=0; j<NB_POS_FCB_2T; j++)
        {
            test();
            if(sub(s_and(pol[ii],pol[jj]),1) == 0 && sub(s_or(pol[ii],pol[jj]),-1) == 0)
            {
                rrixiy[k+j] = negate(rrixiy[k+j]);
                move16();
            }
            jj=add(jj,2);
        }
        ii=add(ii,2);
        k=add(k,NB_POS_FCB_2T);
    }

    /*----------------------------------------------------------------*
    * search 2 pulses
    * All combinaisons are tested:
    * 32 pos x 32 pos x 2 signs = 2048 tests
    *----------------------------------------------------------------*/

    p0 = rrixix[0];
    move16();
    p1 = rrixix[1];
    move16();
    p2 = rrixiy;
    move16();
    psk = -1;
    move16();
    alpk = 1;
    move16();
    ix = 0;
    move16();
    iy = 1;
    move16();

    FOR (i0 = 0; i0 < L_SUBFR; i0 += STEP)
    {
        ps1 = dn_p[i0];
        move16();
        alp1 = *p0++;
        move16();
        pos = -1;
        move16();
        FOR (i1 = 1; i1 < L_SUBFR; i1 += STEP)
        {
            ps2 = add(ps1,dn_p[i1]);
            alp2 = add(alp1, add(*p1++, *p2++));
            sq = mult(ps2, ps2);
            s = L_msu(L_mult(alpk, sq), psk, alp2);
            IF(s > 0)
            {
                psk = sq;
                move16();
                alpk = alp2;
                move16();
                pos = i1;
                move16();
            }
        }
        p1 -= NB_POS_FCB_2T;
        move16();

        IF (pos >= 0)
        {
            ix = i0;
            move16();
            iy = pos;
            move16();
        }
    }

    i0 = shr(ix,1);
    i1 = shr(iy,1);

    sign0 = shl(pol[ix],9);
    sign1 = shl(pol[iy],9);

    /*-------------------------------------------------------------------*
    * Build the codeword, the filtered codeword and index of codevector.
    *-------------------------------------------------------------------*/

    set16_fx( code, 0, L_SUBFR );

    code[ix] = sign0;
    move16();
    code[iy] = sign1;
    move16();

    index = add(shl(i0, 6), i1);


    if (sign0 < 0)
    {
        index = add(index, 0x800);
    }
    if (sign1 < 0)
    {
        index = add(index, 0x20);       /* move16();*/
    }

    set16_fx( y, 0, L_SUBFR );
    /* y_Q9 = sign_Q9<<3 * h_Q12 */

    sign0 = shl(sign0, 3);
    sign1 = shl(sign1, 3);

    FOR(i=ix; i<L_SUBFR; i++)
    {
        y[i] = mult_r(sign0, h[i-ix]);
        move16();
    }
    FOR(i=iy; i<L_SUBFR; i++)
    {
        y[i] = round_fx(L_mac(L_deposit_h(y[i]),sign1,h[i-iy]));
    }
    {
        /* write index to array of indices */
        push_indice_fx( st_fx, IND_ALG_CDBK_2T32, index, 12 );
    }
    return;
}

/*----------------------------------------------------------------------------------
* Function  acelp_1t64()
*
* 7 bits algebraic codebook.
* 1 track x 64 positions per track = 64 samples.
*
* The pulse can have 64 possible positions and two (2) possible amplitudes: +1 or -1.
*----------------------------------------------------------------------------------*/

void acelp_1t64_fx(
    Encoder_State_fx *st_fx,               /* i/o: encoder state structure */
    const Word16 dn[],             /* i  : corr. between target and h[].                 */
    const Word16 h[],              /* i  : impulse response of weighted synthesis filter */
    Word16 code[],           /* o  : algebraic (fixed) codebook excitation         */
    Word16 y[]               /* o  : filtered fixed codebook excitation            */
)
{
    Word16 i, pos, sgn, index;
    Word32 L_tmp;
    /*-------------------------------------------------------------------*
     * Find position and sign of maximum impulse.
     *-------------------------------------------------------------------*/
    pos = emaximum_fx( 0, dn, L_SUBFR, &L_tmp );

    IF(dn[pos]<0)
    {
        sgn = -512;
        move16();
    }
    ELSE
    {
        sgn = 512;
        move16();
    }

    /*-------------------------------------------------------------------*
     * Build the codeword, the filtered codeword and index of codevector.
     *-------------------------------------------------------------------*/

    set16_fx( code, 0, L_SUBFR );
    code[pos] = sgn;
    move16();

    set16_fx( y, 0, L_SUBFR );

    FOR( i=pos; i<L_SUBFR; i++ )
    {
        IF(sgn>0)
        {
            y[i] = shr_r(h[i-pos],3);
            move16();
        }
        ELSE
        {
            y[i] = negate(shr_r(h[i-pos],3));
            move16();
        }
    }

    index = pos;
    move16();
    if( sgn > 0 )
    {
        index = add(index,L_SUBFR);
    }
    {
        push_indice_fx( st_fx, IND_ALG_CDBK_1T64, index, 7 );
    }

    return;
}
