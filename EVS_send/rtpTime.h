#pragma once
#include <windows.h>

class Time
{
public:
	static int getCurrentMilliseconds()
	{
		SYSTEMTIME sys;
		GetLocalTime( &sys );
		int time = sys.wHour*60*60*1000 + sys.wMinute*60*1000
			+ sys.wSecond*1000 + sys.wMilliseconds;
		return time;
	}
};

