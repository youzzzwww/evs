/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */

#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "rom_dec_fx.h"
#include "prot_fx.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * Local function prototype
 *-------------------------------------------------------------------*/
static void fcb_decode_pos_fx(Word16 index, Word16 pos_vector[], Word16 pulse_num, Word16 pos_num);

/*-------------------------------------------------------------------*
 * re8_decode_base_index_fx
 *
 * Decode RE8 base index
 *-------------------------------------------------------------------*/
void re8_decode_base_index_fx(
    Word16 n,  /* i  : codebook number (*n is an integer defined in {0,2,3,4,..,n_max}) */
    UWord16 I,  /* i  : index of c (pointer to unsigned 16-bit word)                     */
    Word16 *x  /* o  : point in RE8 (8-dimensional integer vector)                      */
)
{
    Word16 i,j,k1,l,m,m1,m2;
    Word16 setor_8p_temp[8],setor_8p_temp_1[8],setor_8p_temp_2[8]= {0};
    Word16 sign_8p;
    Word16 code_level;
    const Word16 *a1,*a2;

    Word16 ka;
    UWord16 offset;
    Word16 code_index;

    Word16 element_a10, element_a11 = 0, element_a12 = 0;

    IF (sub( n, 2 ) < 0)
    {
        FOR (i=0; i<8; i++)
        {
            x[i]=0;
            move16();
        }
    }
    ELSE
    {
        if ( L_sub( I, 65519 ) > 0 )
        {
            I = 0;
            move16();
        }

        /*-------------------------------------------------------------------*
         * search for the identifier ka of the absolute leader (table-lookup)
         * Q2 is a subset of Q3 - the two cases are considered in the same branch
         *-------------------------------------------------------------------*/
        IF (sub(n,3) <= 0 )
        {
            FOR (i = 1; i < NB_LDQ3; i++)
            {
                IF (L_sub(I, I3_fx[i]) < 0)
                {
                    BREAK;
                }
            }
            ka = A3_fx[sub(i,1)];
            move16();
        }
        ELSE
        {
            FOR (i = 1; i < NB_LDQ4; i++)
            {
                IF (L_sub(I, I4_fx[i]) < 0)
                {
                    BREAK;
                }
            }
            ka = A4_fx[sub(i,1)];
            move16();
        }

        /*-------------------------------------------------------*
         * decode
         *-------------------------------------------------------*/
        a1 = vals_a_fx[ka];
        move16();
        a2 = vals_q_fx[ka];
        move16();
        k1 = a2[0];
        move16();
        code_level = a2[1];
        move16();

        offset = Is_fx[ka];
        move16();
        code_index = extract_l( L_sub( I, offset ) );

        sign_8p = s_and(code_index, sub( shl(1,k1), 1 ));

        code_index = shr(code_index, k1);

        m = 0;
        move16();
        m1 = 0;
        move16();
        m2 = 0;
        move16();

        element_a10 = a1[0];
        move16();

        SWITCH (code_level)
        {
        case 4:

            m2 = 1;
            move16();
            i = s_and(code_index, 1);
            setor_8p_temp_2[0] = 0;
            move16();

            if ( i )
            {
                setor_8p_temp_2[0] = 1;
                move16();
            }
            code_index = shr(code_index, 1);
            /* no break */

        case 3:

            m = a2[2];
            move16();
            m1 = a2[3];
            move16();

            l = select_table22_fx[m1][m];
            move16();
            j = extract_l(L_shr(L_mult0(code_index, mult_avq_tab_fx[l]), shift_avq_tab_fx[l]));
            code_index = sub(code_index, extract_l(L_mult0(j, l)));
            fcb_decode_pos_fx(code_index,setor_8p_temp_1, m, m1);

            code_index = j;
            move16();
            element_a12 = a1[2];
            move16();
            /* no break */

        case 2:

            m = a2[2];
            move16();
            fcb_decode_pos_fx(code_index,setor_8p_temp,8,m);
            element_a11 = a1[1];
            move16();
        }

        FOR (i=0; i<8; i++)
        {
            x[i] = element_a10;
            move16();
        }

        FOR (i=0; i<m; i++)
        {
            x[setor_8p_temp[i]] = element_a11;
            move16();
        }

        FOR (i=0; i<m1; i++)
        {
            x[setor_8p_temp[setor_8p_temp_1[i]]] = element_a12;
            move16();
        }

        FOR (i=0; i<m2; i++)
        {
            x[setor_8p_temp[setor_8p_temp_1[setor_8p_temp_2[0]]]] = 6;
            move16();
        }

        /*--------------------------------------------------------------------*
         * add the sign of all element ( except the last one in some case )
         *--------------------------------------------------------------------*/
        m1 = sub(k1, 1);

        FOR (i=0; i<8; i++)
        {
            IF (x[i] != 0)
            {
                IF (s_and(shr(sign_8p, m1), 1) != 0)
                {
                    x[i] = sub(0, x[i]);
                    move16();
                }
                m1 = sub(m1, 1);
            }
        }

        /*--------------------------------------------------------------------*
         * recover the sign of last element if needed
         *--------------------------------------------------------------------*/
        IF (sub( k1, 7 ) == 0)
        {
            m1 = 0;
            move16();

            FOR (i=0; i<8; i++)
            {
                m1 = add(m1, x[i]);
            }

            IF ( s_and(m1, 3) )
            {
                x[7] = sub(0, x[7]);
                move16();
            }
        }
    }
}

/*-------------------------------------------------------------------*
 * fcb_decode_pos_fx
 *
 * base function for decoding position index
 *-------------------------------------------------------------------*/
static void fcb_decode_pos_fx(
    Word16 index,             /* i  : Index to decoder    */
    Word16 pos_vector[],      /* o  : Position vector     */
    Word16 pulse_num,         /* i  : Number of pulses    */
    Word16 pos_num            /* i  : Number of positions */
)
{
    Word16 i,k,l;
    Word16 temp1,temp2,tmp_loop;
    const Word16 *select_table23;
    const Word16 *select_table24;

    k = index;
    move16();
    l = 0;
    move16();
    temp1 = pos_num;
    move16();
    temp2 = add( pulse_num, 1 );

    tmp_loop = sub(pos_num,1);
    FOR (i=0; i<tmp_loop; i++)
    {
        select_table23 = select_table22_fx[temp1] ;
        move16();
        select_table24 = &select_table23[sub(pulse_num,l)];
        move16();

        k = sub(*select_table24,k);
        WHILE (sub( k, *select_table24 ) <= 0 )
        {
            l = add(l,1);
            select_table24--;
        }

        k = sub(select_table23[sub(temp2,l)], k);
        pos_vector[i] = sub(l,1);
        move16();
        temp1 = sub(temp1, 1);
    }
    pos_vector[i] = add(l,k);
    move16();
}
