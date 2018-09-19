#include <boost/date_time/posix_time/posix_time.hpp>


#include "TimeStampAbs.h"
#include <windows.h>


boost::uint64_t posix__clock_gettime();
double TimeStampAbs::getTimeStamp(TIME_STAMP_TYPE type) {
	if (type == TIME_STAMP_TYPE::ABS_TIME_STAMP) {
		return getAbsTimeStamp();
	} else if (type == TIME_STAMP_TYPE::TIME_OF_DAY) {
		return getTimeOfDay();
	} else if (type == TIME_STAMP_TYPE::TICK_TIME_STAMP) {
		return getTickTimeStamp();
	} else if (type == TIME_STAMP_TYPE::QUERY_PERFORMANCE) {
		return getQueryPerformanceTime();
	} else {
		return 0;
	}
}

double TimeStampAbs::getAbsTimeStamp() {
	boost::posix_time::ptime epoch(boost::gregorian::date(2016, 10, 1));
	__int64 milli = (boost::posix_time::microsec_clock::universal_time() - epoch).total_milliseconds();
	return (double)milli / 1000.0;
}

double TimeStampAbs::getTimeOfDay() {
	boost::posix_time::ptime systime(boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration td = systime.time_of_day();
	return td.total_microseconds() / 1e6;
}

double TimeStampAbs::getTickTimeStamp() {
	double t = GetTickCount64() / 1000.0;
	return t;
}

double TimeStampAbs::getQueryPerformanceTime() {
	double t = posix__clock_gettime() / 10000000.0;
	return t;
}

#define ET_METHOD_NTKRNL				((boost::uint64_t)1000 * 1000 * 10)
#define ET_METHOD_MILLISECOND			(1000)

boost::uint64_t posix__clock_gettime() {
#if _WIN32
	LARGE_INTEGER counter;
	static LARGE_INTEGER frequency = { 0 };
	if (0 == frequency.QuadPart) {
		if (!QueryPerformanceFrequency(&frequency)) {
			return 0;
		}
	}

	if (QueryPerformanceCounter(&counter)) {
		return (ET_METHOD_NTKRNL * counter.QuadPart / frequency.QuadPart);
	}
	return 0;
#else
	// gcc -lrt
	struct timespec tsc;
	if (clock_gettime(CLOCK_MONOTONIC, &tsc) >= 0) { // CLOCK_REALTIME
		return tsc.tv_nsec / 100; // 返回 100ns, 兼容windows的KRNL计时
	}
	return 0;
#endif
}