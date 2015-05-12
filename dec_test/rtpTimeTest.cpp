#include <gtest\gtest.h>
#include "rtpTime.h"

TEST(TimeTest, getMillisecondTest)
{
	int millisecond = Time::getCurrentMilliseconds();
	EXPECT_GE(millisecond, 0);
	EXPECT_LE(millisecond, 24*60*60*1000);
}
TEST(TimeTest, normalDistributionTest)
{
	double data=0;
	int delay =80;
	FILE* GaussDataFile=fopen("..\\gauss.txt","w+");
	for(int i=1;i<=20;i++)
	{
		for(int j=0;j<100;j++)
		{
			//data = Time::GaussRand();
			data = i*20 + 20*Time::NormalDistrWithScale(1);
			fprintf(GaussDataFile,"%f\n",data);
		}
	}
	fclose(GaussDataFile);
}
