#pragma once
#include "ortp/ortp.h"
#include <time.h>

//extern RtpSession *session;

int rtpInitialize();
int rtpSend(char* buffer, int size);
//DWORD WINAPI rtpSend(LPVOID pParam);
int rtpDestory();
