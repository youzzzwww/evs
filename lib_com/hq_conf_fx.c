/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"
#include "cnst_fx.h"               /* Audio core constants */
#include "rom_com_fx.h"            /* Static table prototypes                */
#include "prot_fx.h"               /* Function prototypes                    */
#include "stl.h"        /* required for wmc_tool */

/*--------------------------------------------------------------------------*
 * hq_configure()
 *
 * Configuration routine for HQ mode
 *--------------------------------------------------------------------------*/

void hq_configure_fx(
    const Word16 length,              /* i  : Frame length                      Q0 */
    const Word16 hqswb_clas,          /* i  : HQ SWB class                      Q0 */
    const Word32 core_brate,          /* i  : Codec bitrate                     Q0 */
    Word16 *num_sfm,            /* o  : Total number of subbands          Q0 */
    Word16 *nb_sfm,             /* o  : Total number of coded bands       Q0 */
    Word16 *start_norm,         /* o  : First norm to be SDE encoded      Q0 */
    Word16 *num_env_bands,      /* o  : Number coded envelope bands       Q0 */
    Word16 *numnrmibits,        /* o  : Number of bits in fall-back norm encoding  Q0 */
    Word16 *hq_generic_offset,  /* o  : Freq offset for HQ GENERIC        Q0 */
    Word16 const **sfmsize,     /* o  : Subband bandwidths                Q0 */
    Word16 const **sfm_start,   /* o  : Subband start coefficients        Q0 */
    Word16 const **sfm_end      /* o  : Subband end coefficients          Q0 */
)
{
    *start_norm = 0;
    move16();

    IF ( sub(length, L_FRAME48k) == 0 )
    {
        IF ( sub(hqswb_clas, HQ_GEN_FB) == 0 )
        {
            *num_sfm = NB_SFM;
            move16();
            *sfmsize = band_len;
            move16();
            *sfm_start = band_start;
            move16();
            *sfm_end = band_end;
            move16();

            test();
            IF ( L_sub(core_brate, HQ_32k) == 0 )
            {
                *hq_generic_offset = HQ_GENERIC_FOFFSET_32K;
                move16();
            }
            ELSE IF ( L_sub(core_brate, HQ_16k40) == 0 || L_sub(core_brate, HQ_24k40) == 0 )
            {
                *hq_generic_offset = HQ_GENERIC_FOFFSET_24K4;
                move16();
            }

            /* setting start frequency of FD BWE */
            test();
            IF ( L_sub(core_brate, HQ_32k) == 0 )
            {
                *num_env_bands = SFM_N_STA_10k;
                move16();
            }
            ELSE IF ( L_sub(core_brate, HQ_16k40) == 0 || L_sub(core_brate, HQ_24k40) == 0 )
            {
                *num_env_bands = SFM_N_STA_8k;
                move16();
            }
            *nb_sfm = *num_sfm;
            move16();
        }
        ELSE
        {
            IF(sub(hqswb_clas, HQ_HARMONIC) == 0)
            {
                *num_sfm = SFM_N_HARM_FB;
                move16();
                *nb_sfm = SFM_N_HARM_FB;
                move16();
                *num_env_bands = SFM_N_HARM_FB;
                move16();

                *sfmsize = band_len_harm;
                move16();
                *sfm_start = band_start_harm;
                move16();
                *sfm_end = band_end_harm;
                move16();
            }
            ELSE IF ( sub(hqswb_clas, HQ_HVQ) == 0 )
            {
                IF ( L_sub(core_brate, HQ_24k40) == 0 )
                {
                    *num_sfm = SFM_N_HARM_FB;
                    move16();
                    *nb_sfm = HVQ_THRES_SFM_24k;
                    move16();
                    *num_env_bands = sub(*num_sfm, *nb_sfm);

                    *sfmsize = band_len_harm;
                    move16();
                    *sfm_start = band_start_harm;
                    move16();
                    *sfm_end = band_end_harm;
                    move16();
                    *start_norm = HVQ_THRES_SFM_24k;
                    move16();
                }
                ELSE
                {
                    *num_sfm = SFM_N_HARM_FB;
                    move16();
                    *nb_sfm = HVQ_THRES_SFM_32k;
                    move16();
                    *num_env_bands = sub(*num_sfm, *nb_sfm);

                    *sfmsize = band_len_harm;
                    move16();
                    *sfm_start = band_start_harm;
                    move16();
                    *start_norm = HVQ_THRES_SFM_32k;
                    move16();
                    *sfm_end = band_end_harm;
                    move16();
                }
            }
            ELSE
            {
                *num_sfm = NB_SFM;
                move16();
                *nb_sfm = *num_sfm;
                move16();
                *num_env_bands = NB_SFM;
                move16();

                *sfmsize = band_len;
                move16();
                *sfm_start = band_start;
                move16();
                *sfm_end = band_end;
                move16();
            }
        }
    }
    ELSE IF( sub(length, L_FRAME32k) == 0 )
    {
        IF ( sub(hqswb_clas, HQ_HARMONIC) == 0 )
        {
            *num_sfm = SFM_N_HARM;
            move16();
            *nb_sfm = SFM_N_HARM;
            move16();
            *num_env_bands = SFM_N_HARM;
            move16();

            *sfmsize = band_len_harm;
            move16();
            *sfm_start = band_start_harm;
            move16();
            *sfm_end = band_end_harm;
            move16();
        }
        ELSE IF ( sub(hqswb_clas, HQ_HVQ) == 0 )
        {
            IF ( L_sub(core_brate, HQ_24k40) == 0 )
            {
                *num_sfm = SFM_N_HARM;
                move16();
                *nb_sfm = HVQ_THRES_SFM_24k;
                move16();
                *num_env_bands = sub(*num_sfm, *nb_sfm);

                *sfmsize = band_len_harm;
                move16();
                *sfm_start = band_start_harm;
                move16();
                *sfm_end = band_end_harm;
                move16();
                *start_norm = HVQ_THRES_SFM_24k;
                move16();
            }
            ELSE
            {
                *num_sfm = SFM_N_HARM;
                move16();
                *nb_sfm = HVQ_THRES_SFM_32k;
                move16();
                *num_env_bands = sub(*num_sfm, *nb_sfm);

                *sfmsize = band_len_harm;
                move16();
                *sfm_start = band_start_harm;
                move16();
                *start_norm = HVQ_THRES_SFM_32k;
                move16();
                *sfm_end = band_end_harm;
                move16();
            }
        }
        ELSE IF ( sub(hqswb_clas, HQ_GEN_SWB) == 0 )
        {
            *num_sfm = SFM_N_SWB;
            move16();
            *sfmsize = band_len;
            move16();
            *sfm_start = band_start;
            move16();
            *sfm_end = band_end;
            move16();

            IF ( L_sub(core_brate, HQ_32k) == 0 )
            {
                *hq_generic_offset = HQ_GENERIC_FOFFSET_32K;
                move16();
            }
            ELSE if ( L_sub(core_brate, HQ_24k40) == 0 )
            {
                *hq_generic_offset = HQ_GENERIC_FOFFSET_24K4;
                move16();
            }

            /* setting start frequency of HQ Generic */
            IF ( L_sub(core_brate, HQ_32k) == 0 )
            {
                *num_env_bands = SFM_N_STA_10k;
                move16();
            }
            ELSE if( L_sub(core_brate, HQ_24k40) == 0 )
            {
                *num_env_bands = SFM_N_STA_8k;
                move16();
            }

            *nb_sfm = *num_sfm;
            move16();
        }
        ELSE
        {
            /* HQ_NORMAL and HQ_TRANSIENT */
            *num_sfm = SFM_N_SWB;
            move16();
            *nb_sfm = *num_sfm;
            move16();
            *num_env_bands = SFM_N_SWB;
            move16();

            *sfmsize = band_len;
            move16();
            *sfm_start = band_start;
            move16();
            *sfm_end = band_end;
            move16();
        }
    }
    ELSE
    {
        *num_sfm = SFM_N_WB;
        move16();
        *nb_sfm = *num_sfm;
        move16();
        *num_env_bands = SFM_N_WB;
        move16();

        *sfmsize = band_len_wb;
        move16();
        *sfm_start = band_start_wb;
        move16();
        *sfm_end = band_end_wb;
        move16();
    }

    *numnrmibits = extract_l(L_mult0(sub(*num_env_bands, 1), NORMI_BITS));

    return;
}
