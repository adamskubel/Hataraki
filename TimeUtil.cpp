#include "TimeUtil.hpp"

using namespace std;

double TimeUtil::AlarmThreshold = 0.5;

void TimeUtil::setNow(timespec & now) 
{		
	clock_gettime(CLOCK_REALTIME, &now);
}

double TimeUtil::timeSince(struct timespec & sinceTime)
{
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	return getTimeDelta(sinceTime,now);
}

void TimeUtil::assertTime(timespec & start, std::string message)
{
	double elapsed = timeSince(start);
	if (elapsed > AlarmThreshold)
	{
		cout << "Threshold exceeded: " << elapsed << " : " << message << endl;
	}
}

double TimeUtil::getTimeDelta(timespec & t0,timespec & t1)
{
	long nsDiff = t1.tv_nsec - t0.tv_nsec;
	long secDiff = t1.tv_sec - t0.tv_sec;

	double totalTime = nsDiff/1000000000.0;
	totalTime += secDiff;

	return totalTime;
}