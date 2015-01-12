/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"
#include "prot_fx.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * swb_hr_nonzero_subband_noise_fill()
 *
 * SWB BWE HR noise filling of zero subbands
 *-------------------------------------------------------------------*/
static void swb_hr_nonzero_subband_noise_fill_fx(
    const Word16 tilt_wb_fx,            /* i  : tilt of wideband signal      */
    Word16 *t_audio_fx,           /* i/o: mdct spectrum                */
    Word16 *bwe_highrate_seed,    /* i/o: seed of random noise         */
    const Word16 N,                     /* i  : length of subband            */
    const Word16 Nsv,                   /* i  : number of subband            */
    Word16 Q_audio
)
{
    Word16 i, j;
    Word16 *ptr_fx;
    Word16 min_bwe_fx, max_bwe_fx, tmpF_fx;
    Word16 tmp;

    IF( sub(tilt_wb_fx, 10240) > 0 )
    {
        FOR( i=0; i<Nsv; i++ )
        {
            min_bwe_fx = 32767;
            move16();
            max_bwe_fx = 0;
            move16();
            tmp = i_mult2(i, N);
            FOR( j=0; j<N; j++ )
            {
                tmpF_fx = abs_s( t_audio_fx[tmp + j] );

                max_bwe_fx = s_max(tmpF_fx, max_bwe_fx);
                if( tmpF_fx > 0 )
                {
                    min_bwe_fx = s_min(tmpF_fx, min_bwe_fx);
                }
            }

            test();
            if( sub(max_bwe_fx, min_bwe_fx) == 0 && sub(min_bwe_fx, shl(1, Q_audio)) > 0 )
            {
                min_bwe_fx = mult_r(min_bwe_fx, 16384);
            }

            ptr_fx = &t_audio_fx[tmp];
            FOR( j=0; j<N; j++ )
            {
                IF( *ptr_fx == 0 )
                {
                    tmp = mult_r(min_bwe_fx, 16384);
                    if( Random( bwe_highrate_seed ) <= 0 )
                    {
                        tmp = negate(tmp);
                    }
                    *ptr_fx = tmp;
                    move16();
                }
                ptr_fx++;
            }
        }
    }

    return;
}

/*-------------------------------------------------------------------*
 * swb_hr_noise_fill()
 *
 * SWB BWE HR noise filling
 *-------------------------------------------------------------------*/
void swb_hr_noise_fill_fx(
    const Word16 is_transient,         /* i  : transient flag          */
    const Word16 spect_start,          /* i  : spectrum start point    */
    const Word16 spect_end,            /* i  : spectrum end point      */
    const Word16 tilt_wb_fx,           /* i  : tilt of wideband signal */
    const Word16 pitch_fx,             /* i  : pitch value             */
    const Word16 nq[],                 /* i  : AVQ nq index            */
    Word16 Nsv,                  /* i  : number of subband       */
    Word16 *bwe_highrate_seed,   /* i/o: seed of random noise    */
    Word16 *t_audio_fx,          /* i/o: mdct spectrum           */
    Word16 Q_audio
)
{
    Word16 i, j, k;
    Word16 pos_start, pos_end, incr;
    Word16 tmp, tmp1, tmp2, tmp3, tmp_exp;
    Word16 alpha_fx, beta_fx;

    IF( is_transient )
    {
        tmp_exp = sub(16, Q_audio);
        FOR( i=0; i<Nsv; i++ )
        {
            IF( nq[i] == 0 )
            {
                tmp = i_mult(i, WIDTH_BAND);
                FOR( j=0; j<WIDTH_BAND; j++ )
                {
                    t_audio_fx[tmp + j] = shr(Random( bwe_highrate_seed ), tmp_exp);
                    move16();/*Q_audio */
                }
            }
        }
    }
    ELSE
    {
        tmp_exp = sub(15, Q_audio);

        Nsv = shr(sub(spect_end, spect_start), 3);
        alpha_fx = 8192;
        move16();/*Q15 */
        IF( sub(tilt_wb_fx, 10240) > 0 )
        {
            beta_fx = 8192;
            move16();/*Q15 */
        }
        ELSE
        {
            IF( sub(6400, pitch_fx) > 0 )
            {
                beta_fx = 8192;
                move16();/*Q15 */
            }
            ELSE
            {
                beta_fx = div_s(1600, pitch_fx); /*Q15+4-4 ->Q15 */
            }
        }

        i = 0;
        move16();
        IF( nq[i] == 0 )
        {
            i = 1;
            move16();
            WHILE( nq[i] == 0 )
            {
                i++;
                move16();
            }

            pos_start = i;
            move16();
            test();
            WHILE( sub(i, Nsv) < 0 && nq[i] != 0 )
            {
                i++;
                move16();
                test();
            }

            pos_end = sub(i, 1);
            move16();

            IF( sub(pos_end, shl(pos_start, 1)) > 0 )
            {
                pos_end = sub(shl(pos_start, 1), 1);
                move16();
            }

            incr = pos_end;
            move16();

            FOR( j = sub(pos_start, 1); j >= 0; j-- )
            {
                tmp = shl(j, 3);
                tmp1 = shl(incr, 3);
                FOR( k=0; k<WIDTH_BAND; k++ )
                {
                    tmp2 = mult_r(alpha_fx, t_audio_fx[tmp1 + k]);
                    tmp3 = mult_r(beta_fx, shr(Random( bwe_highrate_seed ), tmp_exp));
                    t_audio_fx[tmp + k] = add(tmp2, tmp3);
                    move16();
                }

                /*incr = sub(incr, pos_start) < 0 ? pos_end : sub(incr, 1); move16(); */
                IF(sub(incr, pos_start) < 0)
                {
                    incr = pos_end;
                    move16();
                }
                ELSE
                {
                    incr = sub(incr, 1);
                }
            }
        }

        WHILE( sub(i, Nsv) < 0 )
        {
            IF( nq[i] == 0 )
            {
                pos_start = i;
                move16();
                pos_end = i;
                move16();

                FOR( j=pos_start; j<Nsv; j++ )
                {
                    IF( nq[j] != 0 )
                    {
                        i = j;
                        move16();
                        pos_end = j;
                        move16();
                        BREAK;
                    }
                }

                IF( sub(pos_start, pos_end) == 0 )
                {
                    i = Nsv;
                    move16();
                    pos_end = Nsv;
                    move16();
                }

                incr = sub(pos_start, 1);
                move16();


                FOR( j = sub(pos_end, 1); j >= pos_start; j-- )
                {
                    tmp = shl(j, 3);
                    tmp1 = shl(incr, 3);
                    FOR( k=0; k<WIDTH_BAND; k++ )
                    {
                        tmp2 = mult_r(alpha_fx, t_audio_fx[tmp1 + k]);
                        tmp3 = mult_r(beta_fx, shr(Random( bwe_highrate_seed ), tmp_exp));
                        t_audio_fx[tmp + k] = add(tmp2, tmp3);
                        move16();
                    }
                    /*incr = (incr == 0) ? sub(pos_start, 1) : sub(incr, 1); */
                    incr = sub(incr, 1);
                    if(incr < 0)
                    {
                        incr = sub(pos_start, 1);
                    }
                }
            }
            ELSE
            {
                i++;
            }
        }
    }

    swb_hr_nonzero_subband_noise_fill_fx( tilt_wb_fx, t_audio_fx, bwe_highrate_seed, WIDTH_BAND, Nsv, Q_audio );

    return;
}
