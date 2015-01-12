/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "prot_fx.h"        /* Function prototypes                    */
#include "stl.h"            /* required for wmc_tool */
#include "rom_com_fx.h"     /* Static table prototypes                */
#include "cnst_fx.h"        /* Common constants                       */

/* Local constants */
#define THREN2POW           1518500250L

/*--------------------------------------------------------------------------*
 * logqnorm_fx
 *
 * Log quantization for norms of sub-vectors
 *--------------------------------------------------------------------------*/

void logqnorm_fx(
    const Word32 *L_x,      /* i : coefficient vector                         Qx  */
    const Word16 qx,        /* i : Q value of input                               */
    Word16 *k,        /* o : index                                      Q0  */
    const Word16 L,         /* i : codebook length                            Q0  */
    const Word16 N,         /* i : sub-vector size                            Q0  */
    const Word16 hvq_flag   /* i : HVQ flag                                   Q0  */
)
{
    Word16 i, m;
    Word16 coefs_shift, power_shift, temp_shift;
    Word32 L_temp, L_temp1, L_temp2;
    Word16 coefs16[MAX_SFM_LEN_FX];
    UWord16 lsb;

    Word16 offset = add(3,shl(qx,1)); /* 3 + 2*qx */

    lsb = 0U;       /* to avoid compilation warnings */

    L_temp1 = L_deposit_l(1);
    FOR (i=0; i<N; i++)
    {
        L_temp2 = L_abs(L_x[i]);
        L_temp1 = L_max(L_temp1, L_temp2);
    }
    coefs_shift = sub(norm_l(L_temp1), sqac_headroom_fx[N]);
    L_temp = L_deposit_l(0);

    FOR (i=0; i<N; i++)
    {
        coefs16[i] = extract_h(L_shl(L_x[i], coefs_shift));
        L_temp = L_mac0(L_temp, coefs16[i], coefs16[i]);
    }

    if( sub(N, 1) > 0 )
    {
        Mpy_32_16_ss(L_temp, inv_tbl_fx[N], &L_temp, &lsb);
    }
    power_shift = shl(sub(coefs_shift, 16), 1);

    temp_shift = norm_l(L_temp);
    m = add(temp_shift, power_shift);

    L_temp1 = L_add(L_shl(L_temp, temp_shift), lshr(lsb, sub(16, temp_shift)));

    m = add(offset, m);
    test();
    IF( m < 5 && hvq_flag )
    {
        m = shl(m, 1);
        IF( L_sub(L_temp1, 1276901417L /* 2^0.25 Q30 */) < 0 )
        {
            m = add(m, 2);
        }
        ELSE if( L_sub(L_temp1, 1805811301L /* 2^0.75 Q30 */) < 0 )
        {
            m = add(m, 1);
        }
    }
    ELSE
    {
        if ( L_sub(L_temp1, THREN2POW /* 2^0.5 Q30 */) < 0 )
        {
            m = add(m, 1);
        }
        if ( hvq_flag )
        {
            m = add(m, 5); /* offset, 5 extra levels in HVQ codebook */
        }
    }
    *k = s_max(m, 0);
    i = sub(L, 1);
    *k = s_min(*k, i);

    return;
}

void logqnorm_2_fx(
    const Word32 *env_fl,             /* o, Q10  : index */
    const Word16 L,                   /* i  : codebook length */
    const Word16 n_env_band,          /* i  : sub-vector size */
    const Word16 nb_sfm,              /* i  : sub-vector size */
    Word16 *ynrm,
    Word16 *normqlg2,
    const Word32 *thren               /* i, Q10 : quantization thresholds */
)
{
    Word16 i, j, j1, j2;
    Word32 temp, power;

    FOR( i=n_env_band; i < nb_sfm; i++ )
    {
        temp = env_fl[ sub(i,n_env_band) ];
        IF ( L_sub(thren[0], temp) <= 0 )
        {
            *ynrm = 0;
            move16();
        }
        ELSE IF ( L_sub(thren[sub(L,2)], temp) > 0)
        {
            *ynrm = sub(L, 1);
        }
        ELSE
        {
            power = temp;
            move16();
            j1 = 0;
            move16();
            j2 = sub(L, 1);
            WHILE ( sub(sub(j2,j1),1) > 0 )
            {
                j = shr(add(j1 , j2), 1);
                IF ( L_sub(power,thren[j]) >= 0 )
                {
                    j2 = j;
                    move16();
                }
                ELSE
                {
                    j1 = j;
                    move16();
                }
            }
            *ynrm = j2;
            move16();
        }
        *normqlg2 = dicnlg2[*ynrm];
        move16();
        normqlg2++;
        ynrm++;
    }

    return;
}

/*--------------------------------------------------------------------------
 *  calc_norm_fx()
 *
 *  Calculate the norms for the spectral envelope
 *--------------------------------------------------------------------------*/

void calc_norm_fx(
    const Word32 *L_x,                       /* i  : Input vector.                   Qx  */
    const Word16 qx,                         /* i  : Q value of input                    */
    Word16 *norm,                      /* o  : Quantization indices for norms  Q0  */
    Word16 *normlg,                    /* o  : Quantized norms in log2         Q0  */
    const Word16 start_band,                 /* i  : Indice of band to start coding  Q0  */
    const Word16 num_bands,                  /* i  : Number of bands                 Q0  */
    const Word16 *band_len,                  /* i  : Length of bands                 Q0  */
    const Word16 *band_start                 /* i  : Start of bands                  Q0  */
)
{
    Word16 nrm;
    Word16 band;
    Word16 tmp;

    set16_fx(norm, 0, start_band);
    logqnorm_fx(&L_x[band_start[start_band]], qx, &nrm, 32, band_len[start_band], 0);
    norm[start_band] = nrm;
    move16();
    normlg[start_band] = dicnlg2[nrm];
    move16();

    tmp = add(start_band, num_bands);
    FOR (band = add(start_band, 1); band < tmp; band++)
    {
        logqnorm_fx(&L_x[band_start[band]], qx, &nrm, 40, band_len[band], 0);

        norm[band] = nrm;
        move16();
        normlg[band] = dicnlg2[nrm];
        move16();
    }

    return;
}
