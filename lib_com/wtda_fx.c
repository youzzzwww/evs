/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"    /* Static table prototypes                */
#include "stl.h"        /* required by wmc_tool                       */

#include "prot_fx.h"        /* required by wmc_tool                       */
#include "stat_com.h"
/*--------------------------------------------------------------------------*
 *  wtda()
 *
 *  Windowing and time-domain aliasing
 *--------------------------------------------------------------------------*/
static void mvs2s_inv(
    const Word16 *in,
    Word16 *out,
    Word16 L,
    Word16 decimate
)
{
    Word16 i;
    in= in+ shr(sub(decimate,1),1);
    out=out+sub(L,1);
    FOR (i=0; i<L; i++)
    {
        *out=*in;
        move16();
        in+=decimate;
        out--;
    }

}


static void mvs2s_dec(
    const Word16 *in,
    Word16 *out,
    Word16 L,
    Word16 decimate
)
{
    Word16 i;
    in= in+ shr(sub(decimate,1),1);
    FOR (i=0; i<L; i++)
    {
        *out=*in;
        move16();
        in+=decimate;
        out++;

    }

}

static void copy_win(
    Word16 *out_win,
    Word16 nb_zero,
    const Word16 *in_win,
    Word16 win_lenght,
    Word16 nb_one,
    Word16 decimate)
{
    IF (decimate<0)
    {
        set16_fx(out_win,32767,nb_one);
        mvs2s_inv(in_win,out_win+nb_one,win_lenght,negate(decimate));
        set16_fx(out_win+add(win_lenght,nb_one),0,nb_zero);
    }
    ELSE
    {
        set16_fx(out_win,0,nb_zero);
        mvs2s_dec(in_win,out_win+nb_zero,win_lenght,decimate);
        set16_fx(out_win+add(nb_zero,win_lenght),32767,nb_one);
    }
}

void tcx_get_windows_mode1(
    Word16 left_mode,  /* i: overlap mode of left window half          */
    Word16 right_mode, /* i: overlap mode of right window half         */
    Word16  *left_win,               /* o: left overlap window                       */
    Word16  *right_win,              /* o: right overlap window                      */
    Word16  *left_win_int,           /* o: left overlap window                       */
    Word16  *right_win_int,         /* o: right overlap window                      */
    Word16 const L
)
{
    /* Left part */
    IF ( sub(left_mode,MIN_OVERLAP)==0)
    {
        test();
        IF ( sub(L,256)==0 || sub(L,512)==0 )
        {
            copy_win(left_win,R1_25-4*R2_25/7,small_overlap_25,R2_25/7,3*R2_25/7,1);
        }
        ELSE
        {
            copy_win(left_win,R1_48-4*R2_48/7,small_overlap_48,R2_48/7,3*R2_48/7,1);
            copy_win(left_win_int,R1_16-4*R2_16/7,small_overlap_int,R2_16/7,3*R2_16/7,1);
        }
    }
    ELSE IF( sub(left_mode,HALF_OVERLAP)==0)
    {
        test();
        IF ( sub(L,256)==0 || sub(L,512)==0 )
        {
            copy_win(left_win,R1_25-5*R2_25/7,half_overlap_25,3*R2_25/7,2*R2_25/7,1);
        }
        ELSE
        {
            copy_win(left_win,R1_48-5*R2_48/7,half_overlap_48,3*R2_48/7,2*R2_48/7,1);
            copy_win(left_win_int,R1_16-5*R2_16/7,half_overlap_int,3*R2_16/7,2*R2_16/7,1);
        }
    }
    ELSE IF (sub(left_mode, RECTANGULAR_OVERLAP)==0)         /* not needed ? */
    {
        set16_fx(left_win,0,R1_48);
        set16_fx(left_win_int,0,R1_16);
    }
    ELSE IF  (sub(left_mode, ALDO_WINDOW)==0)   /* ALDO */
    {
        test();
        IF  ( sub(L,256)==0 || sub(L,512)==0 )
        {
            Copy(window_256kHz,left_win,R1_25);
        }
        ELSE
        {
            Copy(window_48kHz_fx,left_win,R1_48);
            Copy(window_8_16_32kHz_fx,left_win_int,R1_16);
        }
    }
    ELSE
    {
        /*assert(!"Window not supported");*/
    }

    /* Right part */
    test();
    IF ( sub(right_mode,MIN_OVERLAP)==0 || sub(right_mode,TRANSITION_OVERLAP)==0)
    {
        test();
        IF  ( sub(L,256)==0 || sub(L,512)==0 )
        {
            copy_win(right_win,3*R2_25/7,small_overlap_25,R2_25/7,3*R2_25/7,-1);
        }
        ELSE
        {

            copy_win(right_win,3*R2_48/7,small_overlap_48,R2_48/7,3*R2_48/7,-1);
            copy_win(right_win_int,3*R2_16/7,small_overlap_int,R2_16/7,3*R2_16/7,-1);
        }
    }
    ELSE IF  (sub(right_mode, HALF_OVERLAP)==0)
    {
        test();
        IF  ( sub(L,256)==0 || sub(L,512)==0 )
        {
            copy_win(right_win,2*R2_25/7,half_overlap_25,3*R2_25/7,2*R2_25/7,-1);
        }
        ELSE
        {
            copy_win(right_win,2*R2_48/7,half_overlap_48,3*R2_48/7,2*R2_48/7,-1);
            copy_win(right_win_int,2*R2_16/7,half_overlap_int,3*R2_16/7,2*R2_16/7,-1);
        }
    }
    ELSE IF  (sub(right_mode, RECTANGULAR_OVERLAP)==0)
    {

        set16_fx(right_win,0,R2_48);
        set16_fx(right_win_int,0,R2_16);
    }
    ELSE IF  (sub(right_mode,ALDO_WINDOW)==0)
    {
        test();
        IF  ( sub(L,256)==0 || sub(L,512)==0 )
        {
            Copy(window_256kHz+R1_25,right_win,R2_25);
        }
        ELSE
        {
            Copy(window_48kHz_fx+R1_48,right_win,R2_48);
            Copy(window_8_16_32kHz_fx+R1_16,right_win_int,R2_16);
        }
    }
    ELSE
    {
        /* assert(!"Window not supported");*/
    }
}


void wtda_fx(
    Word16 *new_audio,         /* i  : input audio   Q0 */
    Word16 *Q,                 /* i/o  : Q of input/Output Audio */
    Word32 *wtda_audio,        /* o  : windowed audio  Qout */
    Word16 *old_wtda,          /* i/o: windowed audio from previous frame Qout */
    Word16 *Qold_wtda,
    Word16 left_mode,
    Word16 right_mode,         /* window overlap of current frame (0: full, 2: none, or 3: half) */
    const Word16 L
)
{
    Word16 i, decimate, decay, tmp;
    Word16 n, windecay48;
    Word16 *allsig_l;
    Word16 *allsig_r;
    Word16 win_right[R2_48];
    Word16 win_int_left[R1_16];
    Word16 win_left[R1_48];
    Word16 win_int_right[R2_16];
    Word16 *p1,*p2,*p3,*p4,*p5,*p6;
    Word32 *pa;

    tcx_get_windows_mode1( left_mode,  right_mode, win_left, win_right, win_int_left,win_int_right, L);

    decimate = 1;
    move16(); /* L_FRAME 48k */
    decay = 0;
    move16();
    windecay48 = WINDECAY48;
    move16();
    n = R1_48-R2_48;
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

    IF ( old_wtda== NULL)
    {

        allsig_r = new_audio+n;
        allsig_l = new_audio+sub(n,L);

    }
    ELSE
    {
        /* Rescale signals if Q are not identical */

        IF (sub(*Qold_wtda,*Q)>0)
        {

            Copy_Scale_sig(old_wtda,old_wtda,L,sub(*Q,*Qold_wtda));
            *Qold_wtda=*Q;
            move16();
        }
        ELSE IF (sub(*Qold_wtda,*Q)<0)
        {
            Copy_Scale_sig(new_audio,new_audio,L,sub(*Qold_wtda,*Q));
            *Q=*Qold_wtda;
            move16();
        }

        allsig_r = new_audio+n;
        allsig_l = old_wtda+n;
    }



    IF  ( L == L_FRAME32k )
    {

        /* decimate = 3 */
        /* decay =1 */

        pa=wtda_audio;
        p1=allsig_r+L_FRAME16k-1;
        p2=win_int_right+  3*L_FRAME16k/2-1-WINDECAY16; /*   3*L/2*decimate-decimate+decay-windecay48;*/
        p3=p1+1;
        p4=p2+1;

        p5=win_right+(3*L_FRAME16k/2-1)*3+1-WINDECAY48;
        p6=win_right+(3*L_FRAME16k/2+1)*3-1-1-WINDECAY48;

        FOR (i=0; i<L_FRAME16k-2*N16_CORE_SW; i+=2)
        {
            *pa++=L_negate(L_mac0(L_mult0(*p1--,*p2--), *p3++,*p4++));
            move32();
            *pa++=L_negate(L_mac0(L_mult0(*p1--,*p5), *p3++,*p6));
            move32();
            p5-=decimate;
            p6+=decimate;
            /* Q + 15 */
        }

        FOR (i=0; i<n; i+=2)
        {
            *pa++=L_mult(*p1--, -16384);
            move32();
            *pa++=L_mult(*p1--, -16384);
            move32();
        }

        pa=wtda_audio+L_FRAME16k;
        p1=allsig_l;
        p2=win_left+1;
        p3=new_audio+2*N16_CORE_SW-1;
        p4=win_int_left;

        FOR (i=0; i<n; i+=2)
        {
            *pa++=L_mac0(L_mult0(*p1++,*p2),*p3--,-32768);
            move32();
            *pa++=L_mac0(L_mult0(*p1++,*p4++),*p3--,-32768);
            move32();
            p2+=decimate;
        }

        p5=allsig_l+L_FRAME32k-1-N16_CORE_SW*2;
        p6=win_left+(L_FRAME16k-N16_CORE_SW)*3-1-1;
        p3=win_int_left+L_FRAME16k-N16_CORE_SW-1;

        tmp = sub(shr(L,1),n);
        FOR (i=0; i<tmp; i+=2)
        {
            *pa++=L_msu0(L_mult0(*p1++,*p2), *p5--,*p6);
            move32();
            *pa++=L_msu0(L_mult0(*p1++,*p4++), *p5--,*p3--);
            move32();
            p2+=decimate;
            p6-=decimate;

        }

    }
    ELSE
    {
        pa=wtda_audio;
        p1=allsig_r+sub(shr(L,1),1);
        p2=win_right+  sub(add(sub(i_mult2(i_mult2(3,shr(L,1)),decimate),decimate),decay),windecay48); /*   3*L/2*decimate-decimate+decay-windecay48;*/
        p3=p1+1;
        p4=win_right+ sub(sub(sub(add(i_mult2(i_mult2(3,shr(L,1)),decimate),decimate),decay),windecay48),1);

        tmp = sub(shr(L,1),n);
        FOR (i=0; i<tmp; i++)
        {
            /* wtda_audio[i]=-allsig_r[L/2-i-1]*win_right[3*L/2*decimate-(i+1)*decimate+decay-windecay48]-allsig_r[L/2+i]*win_right[3*L/2*decimate-1+(i+1)*decimate-decay-windecay48];*/
            *pa++=L_msu0(L_msu0(0,*p1--,*p2), *p3++,*p4);
            move32();
            p2-=decimate;
            p4+=decimate;
            /* Q + 15 */
        }

        FOR (i=0; i<n; i++)
        {
            *pa++=L_negate(L_shr(L_deposit_h(*p1--),1));
            move32();
            /*  wtda_audio[i]=-allsig_r[L/2-i-1]; */
        }



        pa=wtda_audio+shr(L,1);
        p1=allsig_l;
        p2=win_left+decay;
        p3=new_audio+sub(n,1);

        FOR (i=0; i<n; i++)
        {
            *pa++=L_mac0(L_mult0(*p1++,*p2), *p3--,-32768);
            move32();
            p2+=decimate;
            /* wtda_audio[i+L/2]=allsig_l[i]*win_left[i*decimate+decay]-new_audio[n-i-1];*/
        }

        p3=allsig_l+sub(sub(L,1),n);
        p4=win_left+sub(sub(i_mult2(sub(L,n),decimate),1),decay);

        tmp = sub(shr(L,1),n);
        FOR (i=0; i<tmp; i++)
        {
            *pa++=L_msu0(L_mult0(*p1++,*p2), *p3--, *p4);
            move32();
            p2+=decimate;
            p4-=decimate;
            /* wtda_audio[i+L/2]=allsig_l[i]*win_left[i*decimate+decay]-allsig_l[L-i-1]*win_left[L*decimate-i*decimate-1-decay];*/
        }



        /* Windowing and foldin for the MDST */
        /*  if (wtda_audio_dst != NULL)
        {




        for (i=n;i<L/2;i++)
        {
        wtda_audio_dst[L/2-i-1]=allsig_l[i]*win_left[i*decimate+decay]+allsig_l[L-i-1]*win_left[L*decimate-i*decimate-1-decay];
        }

        for (i=0;i<n;i++)
        {
        wtda_audio_dst[L/2-i-1]=allsig_l[i]*win_left[i*decimate+decay]+new_audio[n-i-1];
        }
        for (i=L/2-n;i<L/2;i++)
        {
        wtda_audio_dst[L-i-1]=allsig_r[L/2-i-1];
        }

        for (i=0;i<L/2-n;i++)
        {
        wtda_audio_dst[L-i-1]=allsig_r[L/2-i-1]*win_right[3*L/2*decimate-(i+1)*decimate+decay-windecay48]-allsig_r[L/2+i]*win_right[3*L/2*decimate-1+(i+1)*decimate-decay-windecay48];
        }



        }*/
    }

    *Q=add(*Q,15); /* output Q */

    if (old_wtda != NULL)
    {

        Copy(new_audio,old_wtda,L);

    }
    return;

}


