#include "TimeUtil.hpp"

using namespace std;

double TimeUtil::AlarmThreshold = 0.5;



void TimeUtil::setNow(timespec & now) 
{		
#ifdef __linux__
	clock_gettime(CLOCK_REALTIME, &now);
#else
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	now.tv_sec = mts.tv_sec;
	now.tv_nsec = mts.tv_nsec;
#endif
}

double TimeUtil::timeSince(struct timespec & sinceTime)
{
	struct timespec now;
	setNow(now);
	return getTimeDelta(sinceTime,now);
}

double TimeUtil::timeUntil(struct timespec & untilTime)
{
	struct timespec now;
	setNow(now);
	return getTimeDelta(now,untilTime);
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

	double totalTime = nsDiff/NanoSecondsPerSecond;
	totalTime += secDiff;

	return totalTime;
}

void TimeUtil::addTime(timespec & t0, double seconds, timespec & tResult)
{
	double secondComponent = std::floor(seconds);
	double nanoseconds= (seconds - secondComponent)*NanoSecondsPerSecond;
	
	nanoseconds += t0.tv_nsec;
	secondComponent += t0.tv_sec;

	if (nanoseconds > NanoSecondsPerSecond)	
	{
		nanoseconds -= NanoSecondsPerSecond;
		secondComponent++;
	}

	tResult.tv_sec = (long)secondComponent;
	tResult.tv_nsec = (long)nanoseconds;
}