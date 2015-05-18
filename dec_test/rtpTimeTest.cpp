#include <gtest\gtest.h>
#include "rtpTime.h"

TEST(TimeTest, getMillisecondTest)
{
	int millisecond = Time::getCurrentMilliseconds();
	EXPECT_GE(millisecond, 0);
	EXPECT_LE(millisecond, 24*60*60*1000);
}
void guassFileProduce(const char* file_name, int jitter)
{
	double data=0;
	FILE* GaussDataFile=fopen(file_name,"w+");
	for(int i=1;i<=20;i++)
	{
		for(int j=0;j<100;j++)
		{
			//data = Time::GaussRand();
			data = i*jitter + jitter*Time::NormalDistrWithScale(1);
			fprintf(GaussDataFile,"%f\n",data);
		}
	}
	fclose(GaussDataFile);
}
TEST(TimeTest, normalDistributionTest)
{
	std::stringstream ostr;
	std::string filename;
	for(int i=20;i<=50;i=i+10)
	{
		ostr<<i<<".txt";
		ostr>>filename;
		filename = "..\\gauss_"+filename;
		guassFileProduce(filename.c_str(),i);
		ostr.clear();
	}
}
