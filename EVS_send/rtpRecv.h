#pragma once

#define WIN32
#pragma comment(lib,"oRTP.lib")
#include "ortp/ortp.h"
#include <time.h>




int rtpRecvInitalize(int port, int jitter_adapt, int jittcomp);
int rtpRecv(FILE* fin);
//DWORD WINAPI rtpRecv(LPVOID pParam);
int rtpDestory();