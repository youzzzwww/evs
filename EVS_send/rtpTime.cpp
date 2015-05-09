#include "rtpTime.h"

int Time::getCurrentMilliseconds()
{
	SYSTEMTIME sys;
	GetLocalTime( &sys );
	int time = sys.wHour*60*60*1000 + sys.wMinute*60*1000
		+ sys.wSecond*1000 + sys.wMilliseconds;
	return time;
}
double Time::GaussRand(double dExpect, double dVariance)
{
	static double V1, V2, S;
	static int phase = 0;
	double X;
 
	if ( phase == 0 )
	{
		do
		{
		double U1 = (double)rand() / RAND_MAX;
		double U2 = (double)rand() / RAND_MAX;

		V1 = 2 * U1 - 1;
		V2 = 2 * U2 - 1;
		S = V1 * V1 + V2 * V2;
		}while(S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
	{
		X = V2 * sqrt(-2 * log(S) / S);
	}
 
	phase = 1 - phase; 
	return (X*dVariance + dExpect);
}
double Time::GaussRand()
{
	return GaussRand(0, 1);
}
double Time::NormalDistrWithScale(double threshold)
{
	double value=GaussRand();
	while(abs(value)>threshold)
		value = GaussRand();

	return value;
}