#pragma once
#include <windows.h>
#include <math.h>

class Time
{
public:
	static int getCurrentMilliseconds();
	static double GaussRand(double dExpect, double dVariance);
	static double GaussRand();
	static double NormalDistrWithScale(double threshold);
};

