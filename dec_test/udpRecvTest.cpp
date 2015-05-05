#include <gtest\gtest.h>
#include "udpSocket.h"

TEST(udpTest, udpSendTest)
{
	Package* packet_send = new Package("abc");
	UdpTransmission* udp_send = new UdpSend();
	udp_send->setAddr("127.0.0.1", 30998);
	EXPECT_EQ(packet_send->getLength(), 3);
	EXPECT_EQ(udp_send->Send(packet_send), packet_send->getLength());
	udp_send->close();
}
TEST(udpTest, udpSendRecvTest)
{
	Package* packet_send = new Package("abc\0");
	Package* packet_recv = new Package();
	UdpTransmission* udp_send = new UdpSend();
	UdpTransmission* udp_recv = new UdpRecv();
	udp_send->setAddr("127.0.0.1", 30998);
	udp_recv->setAddr("", 30998);

	udp_send->UdpTransmission::Send(packet_send);
	udp_recv->Recv(packet_recv);
	EXPECT_STREQ(packet_recv->getData(), "abc");

	udp_send->Send(new Package("abc"));
	udp_recv->Recv(packet_recv);
	EXPECT_STREQ(packet_recv->getData(), "abc");

	udp_send->close();
	udp_recv->close();
}
//TEST(udpTest, udpRecvToFileTest)
//{
//	FILE* fin = fopen("..\\udpIn.192", "wb+");
//	Package* packet = new Package();
//	UdpTransmission* udp_recv = new UdpRecv();
//	udp_recv->setAddr("", 30998);
//	while(1)
//	{
//		udp_recv->Recv(packet);
//		if(packet->getLength() == 12)
//			break;
//		printf("packet receive, length:%d\n",packet->getLength());
//		packet->writeToFile(fin);
//	}
//	udp_recv->close();
//}

int _tmain(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}