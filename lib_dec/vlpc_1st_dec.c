/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include <assert.h>
#include "prot_fx.h"
#include "stl.h"

extern Word16 const dico_lsf_abs_8b[];

void vlpc_1st_dec(
    Word16 index,    /* input:  codebook index                  */
    Word16 *lsfq)    /* i/o:    i:prediction   o:quantized lsf  */
{
    Word16 i;
    const Word16 *p_dico;



    assert(index < 256);
    p_dico = &dico_lsf_abs_8b[index * M];
    FOR (i = 0; i < M; i++)
    {
        lsfq[i] = add(lsfq[i], *p_dico);
        move16();
        p_dico++;
    }


}
