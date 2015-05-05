#pragma once

#define WIN32
#pragma comment(lib, "oRTP.lib")
#include "ortp/ortp.h"
#include <time.h>

int getCurrentMilliseconds();
//extern RtpSession *session;
int rtpInitialize();
int rtpSend(char* buffer, int size);
//DWORD WINAPI rtpSend(LPVOID pParam);
int rtpDestory();
