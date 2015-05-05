#include "udpSocket.h"

bool UdpTransmission::socketInitial()
{
	WSADATA wsa;  
    WSAStartup(MAKEWORD(2,2),&wsa);
	if((soc = socket(AF_INET,SOCK_DGRAM,0)) <= 0)  
    {  
        wprintf(L"Create socket fail!\n");  
        return false;  
    } 
	return true;
}
void UdpTransmission::Bind()
{
	bind(soc,(struct sockaddr *)&addr,sizeof(addr));
}
void UdpTransmission::setAddr(const char* ip, int port)
{
	this->socketInitial();

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip);  
    addr.sin_port = htons(port); 
	this->Bind();
}
int UdpTransmission::Recv(Package* p)
{
	int iResult=0;
	char buf[2048];
	int len = sizeof(addr);
	iResult = recvfrom(soc, buf, 2048, 0,(struct sockaddr*)&addr, &len);
	if (iResult == SOCKET_ERROR) {
        wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
		return iResult;
    }
	p->setData(buf, iResult);
	return iResult;
}
int UdpTransmission::Send(Package* p)
{
	int iResult=0;
	iResult = sendto(soc, p->getData(), p->getLength(), 0, (struct sockaddr *)&addr, sizeof(addr));
	if (iResult == SOCKET_ERROR) {
        wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
        closesocket(soc);
        WSACleanup();
    }
	return iResult;
}
int UdpTransmission::close()
{
	int iResult = closesocket(soc);
    if (iResult == SOCKET_ERROR) {
        wprintf(L"closesocket failed with error %d\n", WSAGetLastError());
        return iResult;
    }
	WSACleanup();
	return 0;
}
void UdpSend::setAddr(const char* ip, int port)
{
	this->socketInitial();

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip);  
    addr.sin_port = htons(port); 
}
int UdpSend::Send(Package* p)
{
	Package *p_end = new Package();
	p_end->setString(p->getString()+"\0");
	int iResult = UdpTransmission::Send(p_end);
	delete p_end;
	return iResult;
}
void UdpRecv::setAddr(const char* ip, int port)
{
	this->socketInitial();

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);  
    addr.sin_port = htons(port);
	this->Bind();
}