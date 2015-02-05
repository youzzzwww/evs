#include "rtpRecv.h"

RtpSession *session;
//const int port = 8000;
//const int jittcomp=40;
//const int jitter_adapt=0;

//FILE *fp;
int rtpRecvInitalize(int port, int jitter_adapt, int jittcomp)
{
	ortp_init();
	ortp_scheduler_init();
	ortp_set_log_level_mask(ORTP_DEBUG|ORTP_MESSAGE|ORTP_WARNING|ORTP_ERROR);
	session=rtp_session_new(RTP_SESSION_RECVONLY);	
	rtp_session_set_scheduling_mode(session,1);
	rtp_session_set_blocking_mode(session,1);
	rtp_session_set_local_addr(session,"0.0.0.0",port,-1);
	rtp_session_set_connected_mode(session,TRUE);
	rtp_session_set_symmetric_rtp(session,TRUE);
	rtp_session_enable_adaptive_jitter_compensation(session,jitter_adapt);
	rtp_session_set_jitter_compensation(session,jittcomp);
	rtp_session_set_payload_type(session,0);
	rtp_session_set_recv_buf_size(session, 2000);
	rtp_session_signal_connect(session,"ssrc_changed",(RtpCallback)rtp_session_reset,0);
	return 1;
}
int rtpRecv(FILE* fin)
{
	int count=0,seq_num;
	mblk_t *mp=NULL;
	int package_size=0,payload_size=0;
	short rtp_header = 22;
	int ssrc_pad = 0;
	int ts=0,last_recv=0;

	unsigned char data[2000];int data_size=0;
	int packet_start = 0;
	unsigned char *start=NULL;

	while(1)
	{
		mp = rtp_session_recvm_with_ts(session,ts);

		if(mp)
		{
			seq_num = rtp_get_seqnumber(mp);
			count++;
			printf("receive %d packet, seqence number is %d\n",count, seq_num);

			payload_size = rtp_get_payload(mp, &start);
			package_size = payload_size + RTP_FIXED_HEADER_SIZE;
			//if(package_size == 0)
			//	break;
			last_recv = ts;  //记录最后一次接受包
			packet_start = 1;
			data_size=0;

			memcpy(data+data_size, &package_size, sizeof(int));
			data_size += sizeof(int);
			memcpy(data+data_size, &ts, sizeof(int));
			data_size += sizeof(int);

			memcpy(data+data_size, &rtp_header, sizeof(short));
			data_size += sizeof(short);
			memcpy(data+data_size, mp->b_rptr+2, 3*sizeof(short));
			data_size += 3*sizeof(short);
			memcpy(data+data_size, &ssrc_pad, sizeof(int));
			data_size += sizeof(int);
			memcpy(data+data_size, start, payload_size);
			data_size += payload_size;

			fwrite(data, data_size, 1, fin);
		}
		if( ts-last_recv > 160*300 && packet_start ) //超时退出
			break;
		ts += 160;
	}

	return 1;
}
/*DWORD WINAPI rtpRecv(LPVOID pParam)
{

	return 1;
}*/
int rtpDestory()
{
	//fclose(fp);
	rtp_session_destroy(session);
	ortp_exit();
	
	ortp_global_stats_display();
	return 1;
}