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
	FILE* GaussDataFile=fopen("gauss.txt","w+");
	for(int i=0;i<1000;i++)
	{
		//data = Time::GaussRand();
		data = Time::NormalDistrWithScale(2);
		fprintf(GaussDataFile,"%f\n",data);
	}
	fclose(GaussDataFile);
}
