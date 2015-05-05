#include "rtpSend.h"

const char* ipAddr = "127.0.0.1";
const int port = 30998;
RtpSession *session;

int getCurrentMilliseconds()
{
	unsigned int time=0;
	SYSTEMTIME sys;
	GetLocalTime( &sys );
	time = sys.wHour*60*60*1000 + sys.wMinute*60*1000
		+ sys.wSecond*1000 + sys.wMilliseconds;
	return time;
}
//long encoder_handle; //编码器句柄
int BUF_MAX=65536,OUT_BUF_SIZE=1000;
uint32_t user_ts;

int rtpInitialize()
{
	//RTP初始化		
	char *ssrc;
	int clockslide=0;
	int jitter=0;
	user_ts=getCurrentMilliseconds();

	ortp_init();
	ortp_scheduler_init();
	ortp_set_log_level_mask(ORTP_MESSAGE|ORTP_WARNING|ORTP_ERROR);
	session=rtp_session_new(RTP_SESSION_SENDONLY);	
	
	rtp_session_set_scheduling_mode(session,1);
	rtp_session_set_blocking_mode(session,1);
	rtp_session_set_connected_mode(session,TRUE);
	rtp_session_set_remote_addr(session,ipAddr,port);
	rtp_session_set_payload_type(session,0);
	rtp_session_set_seq_number(session, 1);
	ssrc=getenv("SSRC");
	if (ssrc!=NULL) {
		printf("using SSRC=%i.\n",atoi(ssrc));
		rtp_session_set_ssrc(session,atoi(ssrc));
	}
	
	return 1;
}

int rtpSend(char* buffer, int size)
{//if size=0, send a empty packet
	static int count=0;
	static int send_ts=0;
	mblk_t *m;

	m = rtp_session_create_packet(session,RTP_FIXED_HEADER_SIZE,(uint8_t*)buffer,size);
	rtp_session_sendm_with_sendts(session,m,user_ts,send_ts);

	count++;
	printf("send %d packet\n",count);
	user_ts += 20;
	send_ts += 160;
	return 1;
}
/*
DWORD WINAPI rtpSend(LPVOID pParam)
{
	int i,count=0,read_len=0,read_size=0,ret=0;
	uint32_t user_ts=0;
	int seq_num=0;
	waveBuffer *buffer = (waveBuffer*)pParam;
//rtp发送
	while(buffer->recordFlag)
	{
		read_len = get_read_length(encoder_handle);
		if(read_size+read_len <= buffer->size)
		{			
			memset(in, 0,  BUF_MAX*sizeof(char));
			memcpy(in, buffer->dataBuffer+read_size, read_len);
			ret = doEncoder(encoder_handle, in, out);
			//写入wb数据到记录文件
			fwrite(out, ret, 1, wbFile);

			if(ret>0)
			{
				for(i=0;i<4;i++)
				{
					seq_num = rtp_session_get_seq_number(session);
					rtp_session_send_with_ts(session, out+ret/4*i, ret/4, user_ts);					
					count++;				
					printf("send %d packet|current sequence number is:%d\n",count,seq_num);
					user_ts += 160;
				}
			}
			read_size += read_len;
			read_len = get_read_length(encoder_handle);
		}
		else 
		{
			Sleep(20);
		}
	}
	if(!buffer->recordFlag)
	{
		read_len = get_read_length(encoder_handle);
		while(read_size+read_len <= buffer->size)
		{			
			memset(in, 0,  BUF_MAX*sizeof(char));
			memcpy(in, buffer->dataBuffer+read_size, read_len);
			ret = doEncoder(encoder_handle, in, out);
			//写入wb数据到记录文件
			fwrite(out, ret, 1, wbFile);

			if(ret>0)
			{
				for(i=0;i<4;i++)
				{
					seq_num = rtp_session_get_seq_number(session);
					rtp_session_send_with_ts(session, out+ret/4*i, ret/4, user_ts);					
					count++;				
					printf("send %d packet|current sequence number is:%d\n",count,seq_num);
					user_ts += 160;
				}
			}
			read_size += read_len;
			read_len = get_read_length(encoder_handle);
		}
	}
	return 1;
}*/
int rtpDestory()
{
	rtp_session_destroy(session);
	ortp_exit();
	ortp_global_stats_display();
	//释放编解码器
//	if(-1!=encoder_handle) exitEncoder(encoder_handle);
	//关闭记录文件
//	fclose(wbFile);
	return 1;
}