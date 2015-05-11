#pragma once
#include <stdio.h>

#ifdef __cplusplus
extern "C"{
#endif
	int getCurrentMilliseconds();
	int getGaussRand(int delay_jitter);
	int udpRecvBind(int port);
	void udpDataWriteToFile(FILE* fout);
	void rtpToEvs(FILE* fout, unsigned char* start, int size);
	void udpToEvs(FILE* fout);
	void udpClose();

	int pcmToWav(FILE* f_pcm, const char* wav_name, int sampling_rate);
#ifdef __cplusplus
}
#endif