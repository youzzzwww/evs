#include "rtpSend.h"


const char* ipAddr = "127.0.0.1";
const int port = 8000;
RtpSession *session;

//long encoder_handle; //编码器句柄
int BUF_MAX=65536,OUT_BUF_SIZE=1000;


int rtpInitialize()
{
	//RTP初始化		
	char *ssrc;
	int clockslide=0;
	int jitter=0;

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
{
	int seq_num=0,send_size=0;
	static int count=0;
	uint32_t user_ts=0;
//rtp发送
	while(send_size<size)
	{
		
			rtp_session_send_with_ts(session,(unsigned char*)buffer+send_size,1924,user_ts);
			seq_num = rtp_session_get_seq_number(session);
			count++;				
			printf("send %d packet\n",count);

			send_size += 1924;
			user_ts += 160;
		
		
	}
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