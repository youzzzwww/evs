#include "udpToEvs.h"
#include "udpSocket.h"

Package* packet=NULL;
UdpTransmission* udp_recv=NULL;

int getCurrentMilliseconds()
{
	return Time::getCurrentMilliseconds();
}
int getGaussRand(int delay_jitter)
{
	int jitter=0;
	jitter = delay_jitter + delay_jitter/2*Time::NormalDistrWithScale(2);
	return jitter;
}
int udpRecvBind(int port)
{
	packet = new Package();
	udp_recv = new UdpRecv();
	udp_recv->setAddr("", port);
	return 0;
}
void udpToEvs(FILE* fout)
{
	while(1)
	{
		udp_recv->Recv(packet);
		if(packet->getLength() == 12)
			break;
		printf("packet receive, length:%d\n",packet->getLength());
		packet->udpHeadToEvs();
		packet->writeToFile(fout);
	}
}
void udpDataWriteToFile(FILE* fout)
{
	while(1)
	{
		udp_recv->Recv(packet);
		if(packet->getLength() == 12)
			break;
		printf("packet receive, length:%d\n",packet->getLength());
		packet->splitHead("12");
		packet->dataWriteToFile(fout);
	}
}
void udpClose()
{
	udp_recv->close();
	delete udp_recv;
	delete packet;
}
