#pragma once
#include "package.h"
#pragma comment(lib,"ws2_32.lib") 

class UdpTransmission
{
public:
	bool socketInitial();
	void Bind();
	virtual void setAddr(const char* ip, int port);
	int Recv(Package* packet);
	int Send(Package* packet);
	int close();
protected:
	SOCKADDR_IN addr;
	SOCKET soc;
};

class UdpSend:public UdpTransmission
{
	virtual void setAddr(const char* ip, int port);
	int Send(Package* packet);
};
class UdpRecv:public UdpTransmission
{
	virtual void setAddr(const char* ip, int port);
};