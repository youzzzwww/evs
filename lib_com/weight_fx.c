/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"     /* Common constants                       */
#include "rom_com_fx.h"  /* Static table prototypes                */
#include "prot_fx.h"     /* Function prototypes                    */
#include "stl.h"         /* required for wmc_tool */

/*--------------------------------------------------------------------------*
 * sfm2mqb_fx()
 *
 * Map sub-vectors to pbands
 *--------------------------------------------------------------------------*/

static void sfm2mqb_fx(
    Word16 spe[],        /* i  : sub-vectors   */
    Word16 spe2q[],      /* o  : pbands        */
    const Word16 nb_sfm        /* i  : number of norms */
)
{
    Word16 tmp, i;

    /* short groups */
    spe2q[0]  = add(spe[0], 3);
    move16();
    spe2q[1]  = add(spe[1], 3);
    move16();
    spe2q[2]  = add(spe[2], 3);
    move16();
    spe2q[3]  = add(spe[3], 3);
    move16();
    spe2q[4]  = add(spe[4], 3);
    move16();
    spe2q[5]  = add(spe[5], 3);
    move16();
    spe2q[6]  = add(spe[6], 3);
    move16();
    spe2q[7]  = add(spe[7], 3);
    move16();
    spe2q[8]  = add(spe[8], 3);
    move16();
    spe2q[9]  = add(spe[9], 3);
    move16();

    spe2q[10]  = add(shr(add(spe[10], spe[11]),1),4);
    move16();
    spe2q[11]  = add(shr(add(spe[12], spe[13]),1),4);
    move16();
    spe2q[12]  = add(shr(add(spe[14], spe[15]),1),4);
    move16();

    spe2q[13]  = add(shr(add(spe[16], spe[17]),1),5);
    move16();
    spe2q[14]  = add(shr(add(spe[18], spe[19]),1),5);
    move16();

    tmp = 0;
    move16();
    FOR (i=20; i < 24; i++)
    {
        tmp = add(tmp,spe[i]);
    }
    spe2q[15] = add(mult(tmp,8192),6);
    move16();

    tmp = 0;
    move16();
    FOR (i=24; i < 27; i++)
    {
        tmp = add(tmp,spe[i]);
    }
    spe2q[16] = add(mult(tmp,10923),6);
    move16();

    IF (sub(nb_sfm, SFM_N_STA_8k) > 0)
    {
        tmp = 0;
        move16();
        FOR (i=27; i < 30; i++)
        {
            tmp = add(tmp,spe[i]);
        }
        spe2q[17] = add(mult(tmp,10923),6);
        move16();

        IF (sub(nb_sfm, SFM_N_STA_10k) > 0)
        {
            tmp = 0;
            move16();
            FOR (i=30; i < 35; i++)
            {
                tmp = add(tmp,spe[i]);
            }
            spe2q[18] = add(mult(tmp,6553),7);
            move16();

            tmp = 0;
            move16();
            FOR (i=35; i < 44; i++)
            {
                tmp = add(tmp,spe[i]);
            }
            spe2q[19] = add(mult(tmp,3641),8);
            move16();
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * mqb2sfm_fx()
 *
 * Map pbands to sub-vectors
 *--------------------------------------------------------------------------*/

static void mqb2sfm_fx(
    Word16 spe2q[],      /* i  : pbands         */
    Word16 spe[],        /* o  : sub-vectors    */
    const Word16 lnb_sfm       /* i  : number of norms */
)
{
    Word16 i;

    spe[0]  = spe2q[0];
    move16();
    spe[1]  = spe2q[1];
    move16();
    spe[2]  = spe2q[2];
    move16();
    spe[3]  = spe2q[3];
    move16();
    spe[4]  = spe2q[4];
    move16();
    spe[5]  = spe2q[5];
    move16();
    spe[6]  = spe2q[6];
    move16();
    spe[7]  = spe2q[7];
    move16();
    spe[8]  = spe2q[8];
    move16();
    spe[9]  = spe2q[9];
    move16();

    spe[10] = spe2q[10];
    move16();
    spe[11] = spe2q[10];
    move16();

    spe[12] = spe2q[11];
    move16();
    spe[13] = spe2q[11];
    move16();

    spe[14] = spe2q[12];
    move16();
    spe[15] = spe2q[12];
    move16();

    spe[16] = spe2q[13];
    move16();
    spe[17] = spe2q[13];
    move16();

    spe[18] = spe2q[14];
    move16();
    spe[19] = spe2q[14];
    move16();

    FOR (i=20; i < 24; i++)
    {
        spe[i] = spe2q[15];
        move16();
    }

    FOR (i=24; i < 27; i++)
    {
        spe[i] = spe2q[16];
        move16();
    }

    IF (sub(lnb_sfm, SFM_N_STA_8k) > 0)
    {
        FOR (i=27; i < 30; i++)
        {
            spe[i] = spe2q[17];
            move16();
        }

        IF (sub(lnb_sfm, SFM_N_STA_10k) > 0)
        {
            FOR (i=30; i < 35; i++)
            {
                spe[i] = spe2q[18];
                move16();
            }

            FOR (i=35; i < 44; i++)
            {
                spe[i] = spe2q[19];
                move16();
            }
        }
    }

    return;
}

/*--------------------------------------------------------------------------*
 * map_quant_weight_fx()
 *
 * Calculate the quantization weights
 *--------------------------------------------------------------------------*/

void map_quant_weight_fx(
    const Word16 normqlg2[],            /* i  : quantized norms   */
    Word16 wnorm[],               /* o  : weighted norm     */
    const Word16 is_transient,          /* i  : transient flag    */
    const Word16 nb_sfm                 /* i  : number of norms   */
)
{
    Word16 sfm;
    Word16 tmp16;
    Word16 spe2q[NUM_MAP_BANDS];
    Word16 spe[NB_SFM];

    Word16 spe2q_max;
    Word16 spe2q_min;
    Word16 norm_max;
    Word16 shift;
    Word16 sum;
    Word16 k;
    Word16 lnb_sfm,num_map_bands;

    lnb_sfm = NB_SFM;
    move16();
    num_map_bands = NUM_MAP_BANDS;
    move16();

    IF (is_transient != 0)
    {
        FOR (sfm = 0; sfm < lnb_sfm; sfm+=4)
        {
            sum = 0;
            move16();
            FOR (k=0; k < 4; k++)
            {
                sum = add(sum,normqlg2[sfm+k]);
            }
            sum = shr(sum,2);

            FOR (k=0; k < 4; k++)
            {
                spe[sfm +k] = sum;
                move16();
            }
        }
    }
    ELSE
    {
        IF (sub(nb_sfm, NB_SFM) != 0)
        {
            IF (sub(nb_sfm, SFM_N_STA_8k) == 0)
            {
                lnb_sfm = nb_sfm;
                move16();
                num_map_bands = NUM_MAP_BANDS_HQ_24k4;
                move16();
            }
            ELSE IF (sub(nb_sfm, SFM_N_STA_10k) == 0)
            {
                lnb_sfm = nb_sfm;
                move16();
                num_map_bands = NUM_MAP_BANDS_HQ_32k;
                move16();
            }
        }

        FOR (sfm = 0; sfm < lnb_sfm; sfm++)
        {
            spe[sfm] = normqlg2[sfm];
            move16();
        }
    }

    sfm2mqb_fx(spe, spe2q, lnb_sfm);

    FOR (sfm = 0; sfm < num_map_bands; sfm++)
    {
        spe2q[sfm] = sub(spe2q[sfm],10);
        move16();
    }

    /* spectral smoothing */
    FOR (sfm = 1; sfm < num_map_bands; sfm++)
    {
        tmp16 = sub(spe2q[sfm-1],4);
        spe2q[sfm] = s_max(spe2q[sfm], tmp16);
        move16();
    }

    FOR (sfm = num_map_bands-2 ; sfm >= 0 ; sfm--)
    {
        tmp16 = sub(spe2q[sfm+1],8);
        spe2q[sfm] = s_max(spe2q[sfm], tmp16);
        move16();
    }

    FOR (sfm = 0; sfm < num_map_bands ; sfm++)
    {
        spe2q[sfm] = s_max(spe2q[sfm], a_table_fx[sfm]);
        move16();
    }

    /* Saturate by the Absolute Threshold of Hearing */
    spe2q_max = -32768;
    move16();
    spe2q_min = MAX_16;
    move16();

    FOR (sfm = 0; sfm < num_map_bands ; sfm++)
    {
        spe2q[sfm] = sub(sfm_width[sfm], spe2q[sfm]);
        move16();
        spe2q_max = s_max(spe2q[sfm], spe2q_max);
        spe2q_min = s_min(spe2q[sfm], spe2q_min);
    }

    FOR (sfm = 0; sfm < num_map_bands ; sfm++)
    {
        spe2q[sfm] = sub(spe2q[sfm], spe2q_min);
        move16();
    }

    spe2q_max = sub(spe2q_max, spe2q_min);

    norm_max = norm_s(spe2q_max);

    shift = sub(norm_max,13);

    FOR (sfm = 0; sfm < num_map_bands ; sfm++)
    {
        spe2q[sfm] = shl(spe2q[sfm],shift);
        move16();
    }

    mqb2sfm_fx(spe2q,spe,lnb_sfm);

    IF (is_transient != 0)
    {
        FOR (sfm = 0; sfm < lnb_sfm; sfm+=4)
        {
            sum = 0;
            move16();
            FOR (k=0; k < 4; k++)
            {
                sum = add(sum , spe[sfm+k]);
            }
            sum = shr(sum,2);
            FOR (k=0; k < 4; k++)
            {
                spe[sfm +k] = sum;
                move16();
            }
        }
    }

    /* modify the norms for bit-allocation */
    FOR (sfm = 0; sfm < lnb_sfm ; sfm++)
    {
        wnorm[sfm] = add(spe[sfm], normqlg2[sfm]);
        move16();
    }

    return;
}


