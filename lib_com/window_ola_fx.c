/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                  */
#include "prot_fx.h"    /* Function prototypes                    */
#include "rom_com_fx.h" /* Function prototypes                    */
#include "stl.h"

void sinq_fx(
    const Word16 tmp,    /* i  : sinus factor cos(tmp*i+phi)  Q15*/
    const Word16 phi,    /* i  : sinus phase cos(tmp*i+phi)  Q15*/
    const Word16 N,      /* i  : size of output */
    Word16 x[]           /* o  : output vector  Q15*/
)
{
    Word16 i;
    Word16 tmp1, tmp2;
    Word32 L_tmp, A32,tmp_old,tmp_old_old;

    x[0]= phi;
    move16(); /*sin(x) approximated by x; Q15 */
    tmp1 = add(tmp, phi); /*Q15 */
    L_tmp = L_mult(tmp1, tmp1); /*Q31 */
    L_tmp = Mult_32_16(L_tmp, tmp1); /*Q31 */
    L_tmp = Mult_32_16(L_tmp, 5461); /*Q31; division by 6 */
    tmp2 = round_fx(L_tmp); /*Q15 */
    x[1] = sub(tmp1, tmp2);
    move16(); /* sin(x) approximated by (x-x^3/3!); Q15 */
    tmp1 = add(shl(tmp,1), phi); /*Q15 */
    L_tmp = L_mult(tmp1, tmp1); /*Q31 */
    L_tmp = Mult_32_16(L_tmp, tmp1); /*Q31 */
    L_tmp = Mult_32_16(L_tmp, 5461); /*Q31; division by 6 */
    tmp2 = round_fx(L_tmp); /*Q15 */
    x[2] = sub(tmp1, tmp2);
    move16(); /* sin(x) approximated by (x-x^3/3!); Q15 */


    IF ( sub(abs_s(tmp),3) > 0 )
    {
        /*A=(x[2]+x[0])/x[1]=2*cos(tmp); here approximated by 2*(1-tmp^2/2!)  */
        A32 = L_mult0(tmp,tmp); /*Q30 */
        A32 = L_add(L_sub(1073741824, A32), 1073741824); /*Q30 */
    }
    ELSE
    {
        A32 = L_deposit_l(0);
    }

    tmp_old=L_deposit_h(x[2]); /*Q31 */
    tmp_old_old=L_deposit_h(x[1]);  /*Q31 */

    FOR ( i=3; i<N; i++ )
    {
        /*L_tmp = Mult_32_16(A32,(*pt1++)); //30+15-15+1=Q31 */
        /* x[i] = shl(sub(round_fx(L_tmp), shr((*pt2++),1)),1); move16(); //Q15  */
        L_tmp = Mult_32_32(A32,tmp_old); /*Q30 */
        L_tmp= L_sub(L_tmp,L_shr(tmp_old_old,1)); /*Q30 */
        tmp_old_old = L_add(tmp_old, 0); /*Q31   */
        tmp_old= L_shl(L_tmp,1); /*Q31   */
        x[i]=   round_fx(tmp_old);  /*Q15 */
    }

    return;

}

/*--------------------------------------------------------------------------*/
/*  Function  Window_Ola                                                    */
/*  ~~~~~~~~~~~~~~~~~~~~                                                    */
/*                                                                          */
/*  Windowing, Overlap and Add                                              */
/*--------------------------------------------------------------------------*/
/*  Word32    ImdctOut[]  (i)   input                                       */
/*  Word32    auOut[]     (o)   output audio                                */
/*  Word16    Q_sig       (i/o) Q of Input and Output Signal                */
/*  Word32    OldauOut[]  (i/o) audio from previous frame                   */
/*  Word16    Q_old       (i/o)   Q for audio from previous frame             */
/*  Word16    L           (i)   length                                      */
/*--------------------------------------------------------------------------*/

void window_ola_fx(
    Word32 *wtda_audio,
    Word16 *auOut,
    Word16 *Q_sig,     /* in Q IMDCT /out  auOut */
    Word16 *OldauOut,
    Word16 *Q_old, /* in/out Q OldauOut */
    const Word16 L,
    const Word16 right_mode,
    const Word16 left_mode, /* window overlap of current frame (0: full, 2: none, or 3: half) */
    const Word16 use_bfi_win,
    const Word16 oldHqVoicing,
    Word16 *oldgapsynth
)
{
    Word16 i, decimate, decay, temp, temp_len;
    Word16 n, windecay48;
    Word16 win_right[R2_48];
    Word16 win_int_left[R1_16];
    Word16 win_left[R1_48];
    Word16 win_int_right[R2_16];
    Word16 *p1,*p2,*p3,*p4,*p5,*paout;
    Word32 *pa;
    Word32 ImdctOut[2 * L_FRAME48k]; /*for not to overwrite wtda_audio (ImdctOut) by the rescaling. If problem of max RAM, alternative solution is to output the new Q value of ImdctOut after rescaling */
    Word16 SS2[L_FRAME48k-NS2SA(48000, N_ZERO_MDCT_NS)];
    Word16 wret2[L_FRAME48k-NS2SA(48000, N_ZERO_MDCT_NS)];
    Word16 tmp2;

    /* Windows loading */
    tcx_get_windows_mode1(  left_mode,  right_mode, win_left, win_right, win_int_left,win_int_right, L);

    /* need to rescale input (because ImdctOut not full scaled and cause issue in 16 bit conversion) */
    temp = sub(Find_Max_Norm32(wtda_audio, L), 2); /* rescale to temp-1 because left shift after */
    FOR (i = 0; i < L; i++)
    {
        ImdctOut[i] = L_shl(wtda_audio[i],temp);
        move32();
    }
    *Q_sig=add(*Q_sig,temp);


    /* rescaling for overlapp add */
    IF (sub(add(*Q_old,15),*Q_sig)>0)
    {
        Copy_Scale_sig(OldauOut,OldauOut,L,sub(*Q_sig,add(*Q_old,15)));
        *Q_old=sub(*Q_sig,15);

    }
    ELSE IF (sub(add(*Q_old,15),*Q_sig)<0)
    {
        Scale_sig32(ImdctOut,L,sub(add(*Q_old,15),*Q_sig));
        *Q_sig=add(*Q_old,15);
    }

    *Q_sig=*Q_old;  /*fixing output to new Q_old */


    decimate = 1;
    move16(); /* L_FRAME 48k */
    decay = 0;
    move16();
    windecay48 = WINDECAY48;
    move16();
    n=R1_48-R2_48;
    move16();

    test();
    IF (sub(L,L_FRAME32k)==0 || sub(L,L_FRAME16k)==0 )
    {
        decimate = 3;
        move16();
        decay = 1;
        move16();
        n=R1_16-R2_16;
        move16();

        if(sub(L,L_FRAME32k)==0)
        {
            n=2*N16_CORE_SW;
            move16();
        }

    }
    ELSE IF (sub(L,L_FRAME8k)==0)
    {
        decimate = 6;
        move16();
        decay = 2;
        move16();
        n=(R1_16-R2_16)/2;
        move16();

    }
    ELSE IF (sub(L,512)==0)
    {
        windecay48 = WINDECAY48_256;
        move16();
        decimate = 1;
        move16();
        decay = 0;
        move16();
        n=R1_25-R2_25;
        move16();

    }
    ELSE IF (sub(L,256)==0)
    {
        windecay48 = WINDECAY48_256;
        move16();
        decimate = 2;
        move16();
        decay = 0;
        move16();
        n=(R1_25-R2_25)/2;
        move16();
    }


    paout=auOut-n;

    IF( sub(use_bfi_win,1)==0 )
    {

        temp=sub(L,n);

        tmp2 = div_s(1, temp); /*Q15 */
        tmp2 = round_fx(L_shl(L_mult(tmp2, 25736),2)); /*Q15 */

        sinq_fx(shr(tmp2,1), shr(tmp2,2), temp, SS2);

        IF ( sub(L,L_FRAME32k)==0 )
        {
            p4=SS2+sub(shr(L,1),1);
            p1=wret2+sub(shr(L,1),n);
            p2=win_int_left+L_FRAME16k/2-1;
            p3=win_left+(L_FRAME16k/2-1)*3+1;

            temp_len = shr(L,1);
            FOR (i = 0; i < temp_len; i+=2)
            {
                *p1++=div_s(shr(*p4--,5),add(*p2--,10));
                move16();
                *p1++=div_s(shr(*p4--,5),add(*p3,10));   /*Wret2 Q10 */ move16();
                p3-=decimate;
            }
            p4=SS2+sub(temp,1);
            p1=wret2;
            p2=win_left+(L_FRAME16k-N16_CORE_SW)*3-1-1;
            p3=win_int_left+L_FRAME16k-N16_CORE_SW-1;

            temp_len = sub(shr(L,1), n);
            FOR (i = 0; i < temp_len; i+=2)
            {
                *p1++=div_s(shr(*p4--,5),*p2);
                move16();
                *p1++=div_s(shr(*p4--,5),*p3--);
                move16();
                p2-=decimate;
            }

        }
        ELSE
        {

            p2=win_left+add(i_mult2(sub(shr(L,1),1),decimate),decay);
            p1=wret2+sub(shr(L,1),n);
            p4=SS2+sub(shr(L,1),1);

            temp_len = shr(L,1);
            FOR (i = 0; i < temp_len; i++)
            {
                *p1++=div_s(shr(*p4--,5),add(*p2,10));      /*Wret2 Q10 */ move16();
                p2-=decimate;

            }

            p1=wret2;
            p4=SS2+sub(temp,1);


            p2=win_left+sub(sub(i_mult2(sub(L,n),decimate),decay),1);
            temp_len = sub(shr(L,1), n);
            FOR (i = 0; i < temp_len; i++)
            {
                *p1++=div_s(shr(*p4--,5),*p2);
                move16();
                p2-=decimate;

            }

        }


        p1=paout+n;
        pa=ImdctOut+add(shr(L,1),n);

        temp_len = sub(shr(L,1),n);
        FOR (i = 0; i < temp_len; i++)
        {
            *p1++=round_fx(L_shl(*pa++,1));
        }

        /*p1=paout+shr(L,1);*/
        pa=ImdctOut+sub(L,1);
        temp_len = shr(L,1);
        FOR (i = 0; i < temp_len; i++)
        {
            *p1++ = round_fx(L_negate(L_shl(*pa--,1)));
        }


        p1=auOut;
        p2=SS2;

        IF( oldHqVoicing )
        {
            p3=SS2+sub(temp,1);
            p4=oldgapsynth+n;
            FOR (i=0 ; i < temp; i++)
            {
                *p1 =add( mult(*p1,*p2),mult(*p4,*p3));/*   auOut[i]*SS2[i]+ oldgapsynth[i+n]*(SS2[L-n-i-1]);*/ move16();
                p1++;
                p2++;
                p3--;
                p4++;
            }
        }
        ELSE
        {
            p3=wret2;
            p4=OldauOut+n;
            FOR (i=0; i < temp; i++)
            {
                *p1 =add( mult(*p1,*p2),shl(mult(*p4,*p3),5));/*auOut[i]*SS2[i]+ OldauOut[i+n]*(SS2[L-n-i-1])/(wret2[i]+0.01f);;*/ move16();
                p1++;
                p2++;
                p3++;
                p4++;
            }
        }
    }

    IF ( sub( L, L_FRAME32k)==0 )
    {

        IF (use_bfi_win==0)
        {
            /*decimate=3 */
            /*decay=1 */
            p1=paout+n;
            pa=ImdctOut+add(shr(L,1),n);
            /*p3=win_right+       sub(sub(sub(i_mult2(sub(shl(L,1),n),decimate),1),decay),WINDECAY48); */
            p3=win_right+ (2*L_FRAME16k-N16_CORE_SW)*3-1-1-WINDECAY48;
            p5=win_int_right+ (2*L_FRAME16k-N16_CORE_SW)-1-WINDECAY16;
            p4=OldauOut+n;

            temp_len = sub(shr(L,1),n);
            FOR (i = 0; i < temp_len; i+=2)
            {
                *p1++=round_fx(L_add(L_shl(Mult_32_16(*pa++,*p3),1),L_deposit_h(*p4++)));  /* (( Qin + Q15 -15)+1  + ( Qin - 15 + 16))-1 */
                p3-=decimate;
                *p1++=round_fx(L_add(L_shl(Mult_32_16(*pa++,*p5--),1),L_deposit_h(*p4++)));  /* (( Qin + Q15 -15)+1  + ( Qin - 15 + 16))-1 */
                /*  paout[i] = ImdctOut[L/2 + i] * win_right[(2*L_FRAME16k-(n16+(i-n)/2))*decimate-1-decay-windecay48]+OldauOut[i];
                paout[i+1] = ImdctOut[L/2 + i +1] * win_int_right[2*L_FRAME16k-(n16+(i-n)/2)-1-windecay16]+OldauOut[i+1];*/
            }

            pa=ImdctOut+sub(L,1);
            p3=win_right+  (3*L_FRAME16k/2-1)*3+1-WINDECAY48;
            p5=win_int_right+ (3*L_FRAME16k/2)-1-WINDECAY16;

            temp_len =  sub(shr(L,1),n);
            FOR (i = 0; i < temp_len; i+=2)
            {
                *p1++=round_fx(L_sub(L_deposit_h(*p4++), L_shl(Mult_32_16(*pa--,*p5--),1) ));  /* (( Qin + Q15 -15)+1  + ( Qin - 15 + 16))-1 */
                *p1++=round_fx(L_sub(L_deposit_h(*p4++), L_shl(Mult_32_16(*pa--,*p3),1) ));  /* (( Qin + Q15 -15)+1  + ( Qin - 15 + 16))-1 */
                p3-=decimate;
                /* paout[L/2 + i ] = -ImdctOut[L - 1 - i] * win_int_right[(3*L_FRAME16k/2-1-i/2)-windecay16]+OldauOut[i+L/2];
                paout[L/2 + i +1] = -ImdctOut[L - 1 - (i+1)] * win_right[(3*L_FRAME16k/2-1-i/2)*decimate+decay-windecay48]+OldauOut[i+L/2+1]; */
            }
            FOR (i =0; i < n;  i+=2)
            {
                *p1++=round_fx(L_sub(L_deposit_h(*p4++),L_shl(*pa--,1)));
                *p1++=round_fx(L_sub(L_deposit_h(*p4++),L_shl(*pa--,1)));
                /*  paout[L/2 + i +1] = -ImdctOut[L - 1 - (i+1)]+OldauOut[i+L/2+1] ;
                paout[L/2 + i ] = -ImdctOut[L - 1 - i]+OldauOut[i+L/2]; */
            }
        }

        p1=OldauOut+shr(L,1);
        pa=ImdctOut;
        p2=win_int_left+L_FRAME16k/2-1;
        p3=win_left+(L_FRAME16k/2-1)*3+1;

        temp_len = shr(L,1);
        FOR (i = 0; i < temp_len; i+=2)
        {
            *p1++=round_fx(L_negate(L_shl(Mult_32_16(*pa++,*p2--),1)));
            *p1++=round_fx(L_negate(L_shl(Mult_32_16(*pa++,*p3),1)));
            p3-=decimate;
            /* OldauOut[L/2 + i] = -ImdctOut[i] * win_int_left[(L_FRAME16k/2-i/2-1)];
            OldauOut[L/2 + i +1] = -ImdctOut[i+1] * win_left[(L_FRAME16k/2-i/2-1)*decimate+decay]*/;

        }

        p1=OldauOut+n;
        pa=ImdctOut+sub(sub(shr(L,1),1),n);
        p2=win_left+(L_FRAME16k-N16_CORE_SW)*3-1-1;
        p3=win_int_left+L_FRAME16k-N16_CORE_SW-1;
        temp_len = sub(shr(L,1),n);
        FOR (i = 0; i < temp_len; i+=2)
        {
            *p1++=round_fx(L_negate(L_shl(Mult_32_16(*pa--,*p2),1)));
            *p1++=round_fx(L_negate(L_shl(Mult_32_16(*pa--,*p3--),1)));
            p2-=decimate;
            /*  OldauOut[ i] = -ImdctOut[L/2 - 1 - i]  *win_left[(L_FRAME16k-i/2)*decimate-decay-1];
            OldauOut[ i +1] = -ImdctOut[L/2 - 1 - (i +1)] * win_int_left[L_FRAME16k-(i/2)-1];; */
        }

    }
    ELSE
    {
        IF (use_bfi_win==0)
        {

            p1=paout+n;
            pa=ImdctOut+add(shr(L,1),n);
            p3=win_right+sub(sub(sub(i_mult2(sub(shl(L,1),n),decimate),1),decay),windecay48);
            p4=OldauOut+n;

            temp_len = sub(shr(L,1), n);
            FOR (i = 0; i < temp_len; i++)
            {
                *p1++=round_fx(L_add(L_shl(Mult_32_16(*pa++,*p3),1),L_deposit_h(*p4++)));  /* (( Qin + Q15 -15)+1  + ( Qin - 15 + 16))-1 */
                p3-=decimate;
                /*paout[i] = ImdctOut[L/2 + i] * win_right[(2*L-i)*decimate-1-decay-windecay48]+OldauOut[i];*/
            }


            pa=ImdctOut+sub(L,1);
            p3=win_right+sub(add(i_mult2(sub(i_mult2(shr(L,1),3) ,1),decimate),decay),windecay48);
            temp_len = sub(shr(L,1),n);
            FOR (i = 0; i < temp_len; i++)
            {
                *p1++=round_fx(L_sub(L_deposit_h(*p4++), L_shl(Mult_32_16(*pa--,*p3),1) ));  /* (( Qin + Q15 -15)+1  + ( Qin - 15 + 16))-1 */
                p3-=decimate;
                /* paout[L/2 + i] = -ImdctOut[L - 1 - i] * win_right[(3*L/2-1-i)*decimate+decay-windecay48]+OldauOut[i+L/2]; */
            }
            FOR (i =0; i < n; i++)
            {
                *p1++=round_fx(L_sub(L_deposit_h(*p4++),L_shl(*pa--,1)));
                /*  paout[L/2 + i] = -ImdctOut[L - 1 - i] + OldauOut[i+L/2]; */
            }
        }


        p1=OldauOut+shr(L,1);
        pa=ImdctOut;
        p2=win_left+add(i_mult2(sub(shr(L,1),1),decimate),decay);

        temp_len = shr(L, 1);
        FOR (i = 0; i < temp_len; i++)
        {
            *p1++=round_fx(L_negate(L_shl(Mult_32_16(*pa++,*p2),1)));
            p2-=decimate;
            /*OldauOut[L/2 + i] = -ImdctOut[i] * win_left[(L/2-i-1)*decimate+decay]; */
        }

        p1=OldauOut+n;
        pa=ImdctOut+sub(sub(shr(L,1),1),n);
        p2=win_left+sub(sub(i_mult2(sub(L,n),decimate),decay),1);
        temp_len = sub(shr(L,1),n);
        FOR (i = 0; i < temp_len; i++)
        {
            *p1++=round_fx(L_negate(L_shl(Mult_32_16(*pa--,*p2),1)));
            p2-=decimate;
            /* OldauOut[ i] = -ImdctOut[L/2 - 1 - i] * win_left[(L-i)*decimate-decay-1]; */
        }


    }

    p1=OldauOut;
    pa=ImdctOut+sub(shr(L,1),1);
    p2=paout+L;

    FOR (i = 0; i < n; i++)
    {
        *p1=round_fx(L_negate(L_shl(*pa--,1)));
        *p2++=*p1++;
    }

    /*  for (i = 0; i < n; i++)
      {
          OldauOut[ i] = -ImdctOut[L/2 - 1 - i];
      }
      for (i =0; i <n; i++)
      {
          paout[L + i] = OldauOut[i];
      }*/
}




/*---------------------------------------------------------------------*
* core_switching_OLA()
*
* modify window after HQ core decoding
* overlapp ACELP and HQ
*---------------------------------------------------------------------*/
void core_switching_OLA_fx(
    Word16 *mem_over_hp,       /* i  : upsampling filter memory       Qsubrf (except at 8kHz)  */
    const Word16 last_L_frame,       /* i  : last L_frame lengthture             */
    const Word32   output_Fs,          /* i  : output sampling rate                */
    Word16 *synth,             /* i/o: synthesized signal from HQ core     Qsynth*/
    Word16 *synth_subfr_out,   /* i  : synthesized signal from ACELP core  Qsubfr*/
    Word16 *synth_subfr_bwe,   /* i  : synthesized BWE from ACELP core     Qbwe (todo) */
    const Word16 output_frame,        /* i  : output frame length                 */
    Word16* Qsynth,
    Word16* Qsubfr
)
{
    Word16 i, Loverlapp, out_filt_length, filt_delay, L;
    Word16 tmp_buf_switch[SWITCH_MAX_GAP], tmp_buf_switch2[HQ_DELAY_COMP*HQ_DELTA_MAX+2]; /*Q0 */
    const Word16 *pt_sin=NULL,*pt_cos;
    Word16 *pt2,*pt3,*pt;
    const Word16 *pt4,*pt5;
    Word16 tmp,tmp2,temp_len;
    Word16 decimate=0, delta=0; /* initialize just to avoid compiler warnings */
    const Word16 *on_win, *on_win_int;

    /* Scaling (if needed) */
    tmp=s_min(*Qsynth,*Qsubfr);
    Scale_sig(synth, output_frame, sub(tmp,*Qsynth));
    *Qsynth=tmp;
    move16();
    Scale_sig(synth_subfr_out, SWITCH_MAX_GAP, sub(tmp,*Qsubfr));
    Scale_sig(mem_over_hp,NS2SA(16000, DELAY_CLDFB_NS)+2, sub(tmp, *Qsubfr)); /* reScale mem over HP at Qsynth */
    *Qsubfr=tmp;
    move16();

    /* win = window_48kHz_fx;*/
    /* win_int = window_8_16_32kHz_fx;*/
    on_win = one_on_win_48k_fx;
    on_win_int = one_on_win_8k_16k_48k_fx;


    SWITCH (output_frame)
    {
    case L_FRAME8k:
        decimate = 6;
        move16();
        /*decay = 2; move16();*/
        delta = 1;
        move16();
        pt_sin=sin_switch_8;
        BREAK;
    case L_FRAME16k:
        decimate = 3;
        move16();
        /*decay = 1; move16(); */
        delta = 2;
        move16();
        pt_sin=sin_switch_16;
        BREAK;
    case L_FRAME32k:
        decimate = 3;
        move16();
        /*decay = 1; move16(); */
        delta = 4;
        move16();
        pt_sin=sin_switch_32;
        BREAK;
    case L_FRAME48k:
        decimate = 1;
        move16();
        /*decay = 0; move16();*/
        delta = 6;
        move16();
        pt_sin=sin_switch_48;
        BREAK;
    }

    set16_fx(tmp_buf_switch,0,SWITCH_MAX_GAP);
    set16_fx(tmp_buf_switch2,0, HQ_DELAY_COMP*HQ_DELTA_MAX+2);

    Loverlapp = i_mult2(delta,SWITCH_OVERLAP_8k);
    pt_cos=pt_sin+Loverlapp-1;

    Copy( synth_subfr_out,tmp_buf_switch, i_mult2(SWITCH_GAP_LENGTH_8k,delta));  /* copy 6.25 ms subframe  */


    /* conversion from 12.8kHz to output_Fs */
    IF( sub(last_L_frame,L_FRAME)==0)
    {
        /* resample filter memory */
        IF ( sub(output_frame,L_FRAME8k)==0 )
        {
            Copy( synth_subfr_out + SWITCH_GAP_LENGTH_8k, tmp_buf_switch+ SWITCH_GAP_LENGTH_8k, NS2SA(output_Fs, DELAY_CLDFB_NS)); /* copy subframe to tmp buffer */
        }
        ELSE
        {
            filt_delay = 0;
            move16();
            out_filt_length =0;
            move16();
            out_filt_length = modify_Fs_intcub3m_sup_fx( mem_over_hp+2,  NS2SA(12800, DELAY_CLDFB_NS), 12800, tmp_buf_switch2, output_Fs , &filt_delay );
            pt=tmp_buf_switch2+sub(out_filt_length,filt_delay);
            pt2=pt-1;
            FOR( i=0; i<filt_delay; i++ )
            {
                *pt++ = *pt2;
                move16();
            }
            Copy( tmp_buf_switch2, tmp_buf_switch+i_mult2(SWITCH_GAP_LENGTH_8k,delta), out_filt_length); /* copy filter memory to buffer */
        }

    }
    ELSE
    {


        IF( (output_frame-L_FRAME16k)==0 ) /* no resampling */
        {
            Copy(mem_over_hp+2,tmp_buf_switch+i_mult2(SWITCH_GAP_LENGTH_8k,delta), NS2SA(output_Fs, DELAY_CLDFB_NS));
        }
        ELSE
        {
            IF( (output_frame-L_FRAME8k)==0 )  /* not done yet */
            {
                Copy( synth_subfr_out + SWITCH_GAP_LENGTH_8k, tmp_buf_switch+ SWITCH_GAP_LENGTH_8k, NS2SA(output_Fs, DELAY_CLDFB_NS)); /* copy subframe to tmp buffer */
            }
            ELSE
            {
                out_filt_length = modify_Fs_intcub3m_sup_fx( mem_over_hp+2,  NS2SA(16000, DELAY_CLDFB_NS), 16000, tmp_buf_switch2, output_Fs , &filt_delay );
                pt=tmp_buf_switch2+sub(out_filt_length,filt_delay);
                pt2=pt-1;
                FOR( i=0; i<filt_delay; i++ )
                {
                    *pt++ = *pt2;
                    move16();

                }
                Copy( tmp_buf_switch2, tmp_buf_switch+i_mult2(SWITCH_GAP_LENGTH_8k,delta), out_filt_length); /* copy filter memory to buffer */
            }
        }
    }


    /* compensate aldo */
    L =  i_mult2(delta,SWITCH_GAP_LENGTH_8k+ NS2SA(8000, DELAY_CLDFB_NS)) ; /* 6.25 ms gap + 1.25 ms resamp  */
    tmp=sub(L,Loverlapp);


    set16_fx(synth,0,tmp);


    IF( sub(output_frame,L_FRAME32k)==0 )
    {

        pt=synth+tmp;
        pt2=pt+1;
        pt5=on_win+210-1;
        pt4=on_win_int+70-1;
        temp_len = i_mult2(shr(R2_16,2), delta);
        FOR( i=0 ; i<temp_len; i+=2 )
        {
            *pt= shl( mult_r(*pt,*pt5),1);
            move16();/*      // Q14* Q15 + shl  ==> Q15 */
            *pt2 = shl( mult_r(*pt2,*pt4),1);
            move16();/*/= win[(3*L_FRAME16k/2-1-i/2)*decimate+decay-L_FRAME48k*14/20]; */
            pt+=2;
            pt2+=2;
            pt5-=3;
            pt4--;

        }
    }
    ELSE
    {

        pt=synth+tmp;
        pt5=on_win+210-1;
        temp_len = i_mult2(shr(R2_16,2),delta);
        FOR( i=0 ; i< temp_len; i++ )
        {
            *pt=shl( mult_r(*pt,*pt5),1);
            move16();/*  /= win[(3*output_frame/2-1-i)*decimate+decay-L_FRAME48k*14/20]; */
            pt++;
            pt5-=decimate;
            move16();
        }
    }


    pt=synth+tmp;
    move16();/*Q15 */
    pt2=synth_subfr_bwe+tmp-NS2SA(output_Fs, DELAY_CLDFB_NS);
    move16(); /*Q15 */
    pt3=tmp_buf_switch+tmp;
    move16(); /*Q15 */

    FOR( i=0; i<Loverlapp; i++ ) /*Windowing for overlapadd  */
    {
        *pt = mult_r(*pt,*pt_sin);
        move16();
        pt_sin++;
        tmp2= mult_r(*pt_cos,*pt_cos);
        pt_cos--;
        *pt2=mult_r(*pt2,tmp2);
        move16();
        *pt3=mult_r(*pt3,tmp2);
        move16();
        pt++;
        pt2++;
        pt3++;
    }

    pt=synth;
    pt2=tmp_buf_switch;
    tmp=NS2SA(output_Fs, DELAY_CLDFB_NS);
    move16();
    pt3=synth_subfr_bwe;

    /* overlapadd ACELP (tmp_buf_switch) + HQ (synth) */  /*Q15 */
    FOR( i=0; i<tmp; i++ )
    {
        *pt = add(*pt,*pt2++);
        move16();
        pt++;
    }

    temp_len = sub(L,tmp);
    FOR( i=0; i< temp_len; i++ )
    {
        *pt = add(add(*pt,*pt2++),*pt3++);
        move16();
        pt++;
    }

    return;
}

