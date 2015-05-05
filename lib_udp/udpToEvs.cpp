#include "udpToEvs.h"
#include "udpSocket.h"

Package* packet=NULL;
UdpTransmission* udp_recv=NULL;

int getCurrentMilliseconds()
{
	return Time::getCurrentMilliseconds();
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
		packet->headToEvs();
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
		packet->splitHead(12);
		packet->dataWriteToFile(fout);
	}
}
void udpClose()
{
	udp_recv->close();
	delete udp_recv;
	delete packet;
}
