#pragma once
#include <string>
#include <stdio.h>
#include "rtpTime.h"

class Data
{
public:
	Data(){}
	Data(char* p):data(p){}
	void setData(const char* p)
	{this->data = p;}
	void setData(const char* p, int size)
	{this->data.assign(p, size);}
	const char* getData()
	{return this->data.data();}

	void setString(std::string p)
	{this->data = p;}
	std::string getString()
	{return this->data;}
	int getLength()
	{return data.length();}

	void writeToFile(FILE* fout);
	int readFromFile(FILE* fin, int size);
protected:
	std::string data;
};

class Header:public Data
{
public:
	Header(){}
	void setPayloadSize(int size)
	{this->payload_size = size;}

	template <typename T> void addHead(T head);
	void addEvsHead();
	virtual void toEvs(){}
	virtual void setJitter(int time){}
	virtual void simulatorDelay(int delay){}
private:
	int packet_size;
	int recv_time;
	//short rtp_header;
	//short seqence_number;
	//int time_stamp;
	//int ssrc;
	int payload_size;
};
class RawHeader:public Header
{
public:
	RawHeader(){}
};
class EvsHeader:public Header
{
public:
	EvsHeader(){}
	virtual void setJitter(int time);
	virtual void simulatorDelay(int delay);
	std::string getRecvTime()
	{return this->data.substr(4,4);}
	std::string getTimeStamp()
	{return this->data.substr(12,4);}
	void setRecvTime(const char* recv_time)
	{this->data.replace(4,4,recv_time,4);}
private:
	int delay;     //network delay simulating in milliseconds
};
class UdpHeader:public Header
{
public:
	UdpHeader(){}

	virtual void toEvs();
	std::string getRtpHeader()
	{return this->data.substr(0,2);}
	std::string getSeqenceNumber()
	{return this->data.substr(2,2);}
	std::string getTimeStamp()
	{return this->data.substr(4,4);}
	std::string getSsrc()
	{return this->data.substr(8,4);}
};

class Package:public Data
{
public:
	Package(){this->header=NULL;}
	Package(char* p):Data(p)
	{this->header=NULL;}
	~Package() 
	{
		if(this->header)
			delete this->header;
	}

	int addHeader(Header* head);
	void splitHead(std::string head_type);
	void udpHeadToEvs();
	void compose();
	void dataWriteToFile(FILE* fout);
private:
	void splitHead(int head_size);
public:
	Header* header;
	Data payload;
};