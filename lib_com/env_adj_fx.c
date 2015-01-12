/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "rom_com_fx.h"  /* Static table prototypes                */
#include "prot_fx.h"     /* Function prototypes                    */
#include "stl.h"         /* required by wmc_tool                   */

/*--------------------------------------------------------------------------*
 * env_adj()
 *
 * Adjust the band energies of noise-fill and low resolution bands
 *--------------------------------------------------------------------------*/
void env_adj_fx
(
    const Word16 *pulses,            /* i  : number of pulses per band           Q0  */
    const Word16 length,             /* i  : length of spectrum                  Q0  */
    const Word16 last_sfm,           /* i  : index of the last band              Q0  */
    Word16 *adj,               /* o  : adjustment factors for the envelope Q15 */
    const Word16 env_stab,           /* i  : envelope stability                  Q15 */
    const Word16 *sfmsize            /* i  : subband sizes                       Q0  */
)
{
    Word16 i, j, group;
    Word16 npul;
    Word16 att_state;
    Word16 start, len;
    Word16 tmp, tmp_diff;
    Word16 gain_adj;
    Word16 idx;

    att_state = 0;
    move16();
    len  = 0;
    move16();
    start = 0;
    move16();

    /* Find attenuation levels */
    FOR( i = 0; i <= last_sfm ; i++ )
    {
        group = sub(shr(sfmsize[i],3),1);
        npul  = pulses[i];
        move16();

        IF( sub(length, L_FRAME32k) == 0 )
        {

            IF( npul == 0 )
            {
                /* Noise filled band */
                IF ( sub(group,1) <= 0 )
                {
                    test();
                    test();
                    test();
                    test();
                    IF ( i > 0 && pulses[i-1] != 0 && pulses[i+1] != 0 )
                    {
                        adj[i] = 11796; /* Q15, 0.36f */     move16();
                    }
                    ELSE IF ( i > 0 && ( pulses[i-1] == 0 || pulses[i+1] == 0) )
                    {
                        adj[i] = 17695; /* Q15, 0.54f */     move16();
                    }
                    ELSE
                    {
                        adj[i] = 23593; /* Q15, 0.72f */     move16();
                    }
                }
                ELSE IF ( sub(i,last_sfm) < 0 )
                {
                    test();
                    IF ( pulses[i-1] != 0 && pulses[i+1] != 0 )
                    {
                        adj[i] = 17695; /* Q15, 0.54f */     move16();
                    }
                    ELSE
                    {
                        adj[i] = 23593; /* Q15, 0.72f */     move16();
                    }
                }
                ELSE
                {
                    adj[i] = 23593; /* Q15, 0.72f */     move16();
                }

                if( att_state == 0 )
                {
                    start = i;
                    move16();
                }

                len = add(len,1);
                move16();
                att_state = 1;
                move16();
            }
            ELSE
            {
                adj[i] = MAX_16; /* Q15, 1.0f (saturated) */
                IF( sub(att_state, 1) == 0 ) /* End of attenuation region found */
                {
                    /* tmp = min(1, max(0, len-ENV_ADJ_START)*(1.0f/ENV_ADJ_INCL)); */
                    tmp = round_fx(L_shl(L_mult0(s_max( 0, sub(len, ENV_ADJ_START_FX)), ENV_ADJ_INV_INCL_FX),16)); /* Q15 (15+16-16) */
                    tmp_diff = sub(MAX_16, tmp); /* Q15 */ move16();
                    FOR( j = start; j < i ; j++ )
                    {
                        /* adj[j] = max(tmp + (1-tmp)*adj[j],env_stab); */
                        adj[j] = s_max(add(tmp, mult(tmp_diff, adj[j])), env_stab); /* Q15 (15+15-15) */ move16();
                    }
                    len = 0;
                    move16();
                    att_state = 0;
                    move16();
                }
            }
        }
        /* length == L_FRAME16k */
        ELSE
        {

            /* Calculate low accuracy band attenuation */
            gain_adj = 32767;   /* Q15, 1.0f (saturated) */      move16();

            test();
            IF( npul > 0 && sub(npul, MAX_P_ATT) < 0 )
            {
                /*idx = (short)(npul * att_step[group] + 0.5f) - 1; */
                idx = sub(mult_r(shl(npul,2),att_step_fx[group]), 1); /* Q0 (2+13+1-16) */
                if( sub(idx, MAX_P_ATT) < 0 )
                {
                    gain_adj = gain_att_fx[idx];   /* Q15 */  move16();
                }
            }
            adj[i] = gain_adj;
            move16();
        }
    }

    /* Check if the sequence ended with an attenuation region */
    IF( sub(att_state, 1) == 0 )
    {
        /* tmp = min(1, max(0, len-ENV_ADJ_START)*(1.0f/ENV_ADJ_INCL)); */
        tmp = round_fx(L_shl(L_mult0(s_max( 0, sub(len, ENV_ADJ_START_FX)), ENV_ADJ_INV_INCL_FX),16)); /* Q15 (15+16-16) */
        tmp_diff = sub(MAX_16, tmp); /* Q15 */ move16();
        FOR( j = start; j < i ; j++ )
        {

            /* adj[j] = max(tmp + (1-tmp)*adj[j],env_stab); */
            adj[j] = s_max(add(tmp, mult(tmp_diff, adj[j])), env_stab); /* Q15 (15+15-15) */ move16();
        }
    }

    return;
}
