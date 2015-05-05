#include <gtest\gtest.h>
#include "package.h"

TEST(TimeTest, getMillisecondTest)
{
	int millisecond = Time::getCurrentMilliseconds();
	EXPECT_GE(millisecond, 0);
	EXPECT_LE(millisecond, 24*60*60*1000);
}
TEST(packageTest, headerAddTest)
{
	UdpHeader* head = new UdpHeader();
	head->setData("l");
	Package* packet = new Package("abc");
	EXPECT_EQ(packet->addHeader(head), 1);
	EXPECT_STREQ(packet->header.getData(), "l");
	EXPECT_STREQ(packet->payload.getData(), "abc");
	EXPECT_STREQ(packet->getData(),"labc");

}
TEST(packageTest, headerSplitTest)
{
	Package* packet = new Package("abcdefg");
	packet->splitHead(4);
	EXPECT_STREQ(packet->header.getData(), "abcd");
	EXPECT_STREQ((packet->payload).getData(), "efg");
}
TEST(packageTest, packageContentTest)
{
	Package* packet = new Package();
	packet->setData("abc");
	EXPECT_STREQ(packet->getData(), "abc");
	packet->setData("abcdef",4);
	EXPECT_STREQ(packet->getData(), "abcd");
	Package* packet_2 = new Package("abcd");
	EXPECT_STREQ(packet_2->getData(), "abcd");
}
TEST(packageTest, headerTransferTest)
{
	FILE* fin = fopen("..\\udpIn.192", "rb");
	Package* packet = new Package();
	packet->readFromFile(fin, 1936);
	packet->headToEvs();
	fclose(fin);
}
TEST(packageTest, headerTransferWriteTest)
{
	FILE* fin = fopen("..\\udpIn.192", "rb");
	FILE* fout = fopen("..\\udpOut.192", "wb");
	Package* packet = new Package();
	while( packet->readFromFile(fin, 1936) )
	{
		packet->headToEvs();
		packet->writeToFile(fout);
	}
	fclose(fin);
	fclose(fout);
}
TEST(packageTest, packageFileTest)
{
	FILE* fout = fopen("ftest.txt","w+");
	Package* packet = new Package("abc");
	packet->writeToFile(fout);
	fclose(fout);

	FILE* fb = fopen("ftest", "wb+");
	packet->writeToFile(fb);
	char buffer[4];
	buffer[3]='\0';
	fseek(fb, 0, SEEK_SET);
	fread(buffer, sizeof(char), 3, fb);
	EXPECT_STREQ(buffer, "abc");
	fclose(fb);

	FILE* fin = fopen("ftest", "rb");
	packet->readFromFile(fin, 3);
	EXPECT_STREQ(packet->getData(), "abc");
	fclose(fin);
}