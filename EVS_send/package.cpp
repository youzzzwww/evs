#include "package.h"

void Data::writeToFile(FILE* fin)
{
	fwrite(this->getData(), this->getLength(), 1, fin);
}
int Data::readFromFile(FILE* fin, int size)
{
	char* file_buf = new char[size+1];
	memset(file_buf, 0, size+1);
	int read_size = fread(file_buf, 1, size, fin);
	file_buf[size] = '\0';
	this->setData(file_buf,size);
	delete file_buf;
	return read_size;
}
template <typename T>
void Header::addHead(T head)
{
	int head_size = sizeof(T);
	char* head_str = new char[head_size+1];
	memcpy(head_str, &head, head_size);
	head_str[head_size] = '\0';
	this->setString(std::string(head_str,head_size) + this->data);
}
void Header::addEvsHead()
{
	int evs_header_size = 24;
	this->packet_size = (evs_header_size + this->payload_size)/2;
	this->recv_time = Time::getCurrentMilliseconds();
	this->addHead(this->recv_time);
	this->addHead(this->packet_size);
}
void UdpHeader::toEvs()
{
	unsigned short rtp_header = 22;
	this->setString(getSeqenceNumber()
		+getTimeStamp()+getSsrc());
	this->addHead(rtp_header);
	this->addEvsHead();
}
int Package::addHeader(UdpHeader* head)
{
	this->header = *head;
	this->payload.setString(this->data);
	this->data = head->getString() + data;
	this->header.setPayloadSize(this->payload.getLength());
	return head->getLength();
}
void Package::splitHead(int head_size)
{
	this->header.setString(this->data.substr(0,head_size));
	this->payload.setString(this->data.substr(head_size));
	this->header.setPayloadSize(this->payload.getLength());
}
void Package::headToEvs()
{
	this->splitHead(12);
	this->header.toEvs();
	this->compose();
}
void Package::compose()
{
	this->data = this->header.getString() + this->payload.getString();
}
void Package::dataWriteToFile(FILE* fout)
{
	this->payload.writeToFile(fout);
}