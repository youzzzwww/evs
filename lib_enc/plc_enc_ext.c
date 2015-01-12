/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/


#include <stdlib.h>

#include "stl.h"
#include "cnst_fx.h"
#include "stat_enc_fx.h"
#include "prot_fx.h"

#define NBITS_GACELP 5

extern const Word16 lsf_init[16];


void open_PLC_ENC_EVS(
    HANDLE_PLC_ENC_EVS hPlcExt,
    Word32 sampleRate           /* core coder SR */
)
{
    Word16 itr;


    hPlcExt->enableGplc = 0;
    move16();
    hPlcExt->nBits = NBITS_GACELP;
    move16();

    hPlcExt->Q_exp = 0;
    move16();
    hPlcExt->Q_new = 0;
    move16();

    set16_fx(hPlcExt->mem_MA_14Q1,0,M);
    set16_fx(hPlcExt->mem_AR,0,M);
    set16_fx(hPlcExt->lsfold_14Q1,0,M);
    set16_fx(hPlcExt->lspold_Q15,0,M);

    set16_fx(hPlcExt->old_exc_Qold,0,8);

    set16_fx(hPlcExt->lsfoldbfi0_14Q1,0,M);
    set16_fx(hPlcExt->lsfoldbfi1_14Q1,0,M);
    set16_fx(hPlcExt->lsf_adaptive_mean_14Q1,0,M);
    hPlcExt->stab_fac_Q15 = 0;
    move16();
    IF( L_sub(sampleRate,12800)==0 )
    {
        hPlcExt->T0_4th = L_SUBFR;
        move16();
        hPlcExt->T0 = L_SUBFR;
        move16();
        FOR( itr=0; itr<M; itr++ )
        {
            hPlcExt->lsf_con[itr] = lsf_init[itr];
            move16();
            hPlcExt->last_lsf_ref[itr] = lsf_init[itr];
            move16();
            hPlcExt->last_lsf_con[itr] = lsf_init[itr];
            move16();
        }
    }
    ELSE
    {
        hPlcExt->T0_4th = L_SUBFR;
        move16();
        hPlcExt->T0 = L_SUBFR;
        move16();
        FOR( itr=0; itr<M; itr++ )
        {
            hPlcExt->lsf_con[itr] = add(lsf_init[itr],shr(lsf_init[itr],2));
            hPlcExt->last_lsf_ref[itr] = add(lsf_init[itr],shr(lsf_init[itr],2));
            hPlcExt->last_lsf_con[itr] = add(lsf_init[itr],shr(lsf_init[itr],2));
        }
    }

    return;
}


/*
  function to extract and write guided information
*/
void gPLC_encInfo (HANDLE_PLC_ENC_EVS self,
                   Word32 modeBitrate,
                   Word16 modeBandwidth,
                   Word16 old_clas,
                   Word16 coder_type
                  )
{


    IF (self)
    {
        test();
        test();
        test();
        test();
        test();
        IF ( ( sub(modeBandwidth, WB) == 0  && L_sub(modeBitrate, 24400) == 0 ) ||
             ( sub(modeBandwidth, SWB) == 0 && L_sub(modeBitrate, 24400) == 0 ) ||
             ( sub(modeBandwidth, FB) == 0 && L_sub(modeBitrate, 24400) == 0 ) )
        {
            self->enableGplc = 1;
            move16();
            self->nBits = 1;
            move16();
            test();
            test();
            test();
            if( (sub(old_clas, VOICED_CLAS)==0 || sub(old_clas, ONSET)==0) &&
                    (sub(coder_type, VOICED)==0 || sub(coder_type, GENERIC)==0 ) )
            {
                self->nBits = NBITS_GACELP;
                move16();
            }
        }
        ELSE
        {
            self->enableGplc = 0;
            move16();
            self->nBits = NBITS_GACELP;
            move16();
        }

    }

}

