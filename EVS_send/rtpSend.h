#pragma once

#define WIN32
#pragma comment(lib, "oRTP.lib")
#pragma comment(lib, ".\\Debug\\lib_udp.lib")
#include "ortp/ortp.h"
#include <time.h>

int getCurrentMilliseconds();
//extern RtpSession *session;
int rtpInitialize();
int rtpSend(char* buffer, int size, FILE* f_stream);
//DWORD WINAPI rtpSend(LPVOID pParam);
int rtpDestory();
